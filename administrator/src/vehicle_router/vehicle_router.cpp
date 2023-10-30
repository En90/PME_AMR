#include "administrator/vehicle_router/vehicle_router.h"

namespace administrator{
    Vehicle_Router::Vehicle_Router() : nh("~"){
        //Load param

        //Load plugin
        std::string plugin_("vrp_plugins::Simple"); // can be load as ros param
        try{
            pluginlib::ClassLoader<vrp_base::VehicleRoutingSolver_base> vrp_loader("administrator", "vrp_base::VehicleRoutingSolver_base");
            VRP_Solver = vrp_loader.createInstance(plugin_);
            VRP_Solver->initialize(nh);
        }
        catch(pluginlib::PluginlibException& ex){
            ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        }

        //Init Sub and Pub
        order_sub = nh.subscribe("/confirmed_order", 10, &Vehicle_Router::order_callback, this);
        robot_sub = nh.subscribe("/robot_state", 10, &Vehicle_Router::robot_callback, this);
        site_location_sub = nh.subscribe("/site_location", 50, &Vehicle_Router::site_location_callback, this);
        order_state_pub = nh.advertise<std_msgs::Int16MultiArray>("/order_state", 1);
        mission_pub = nh.advertise<Interface>("/Interface", 1);
        ROS_INFO("Init vehicle router successed");
    }

    Vehicle_Router::~Vehicle_Router(){
    
    }

    void Vehicle_Router::order_callback(const order_msgs::ConstPtr& msg){
        Order order(
            msg->order_id.data,
            msg->missionType.data,
            msg->random_password.data,
            msg->recipient.data,
            msg->recipient_location.data,
            msg->sender.data,
            msg->sender_location.data,
            msg->priority.data);
        ROS_INFO("Add %s", order.order_id.c_str());
        VRP_Solver->waiting_order.emplace(order);
        ROS_INFO("%ld orders in waiting.", VRP_Solver->waiting_order.size());
        VRP_Solver->solve();
        // 激發 若在機器在IDLE的狀態則發一個goal喚醒
        for(std::size_t robot_id = 0; robot_id < VRP_Solver->robots.size(); robot_id++){
            auto& robot_ = VRP_Solver->robots[robot_id];
            if(robot_.state == Robot::State::IDLE && !robot_.goals.empty()){
                std::string order_id = robot_.goals.front().first;
                std::string site_id = robot_.goals.front().second;
                if(send_goal(robot_, robot_id, site_id, order_id)){
                    robot_.goals.pop_front();
                    robot_.state = Robot::State::RUN;
                    robot_.doing_order_id = order_id;
                    robot_.doing_mission = site_id;
                }
                else
                    ROS_ERROR("send goal error");
            }
            else{
                // 已經醒了
            }
        }
    }

    void Vehicle_Router::robot_callback(const RobotState& msg){
        ROS_INFO("get robot state");
        
        // 確認收到的是哪台機器人的 state
        int robot_id = msg.robot_id.data;
        if (robot_id >= VRP_Solver->robot_num)
            ROS_ERROR("error when get robot state: invalid robot id");
        Robot& robot_ = VRP_Solver->robots[robot_id];
        
        // 更新 state
        Robot::State now_state;
        static std::unordered_map<std::string, Robot::State> const table = { {"GOAL",Robot::State::WAIT}, {"STUCK",Robot::State::STUCK} };
        auto it = table.find(msg.state.data);
        if (it != table.end())
            now_state = it->second;
        else
            ROS_ERROR("error when get robot state: invalid state");
        
        // 針對 state 做出反應
        if(now_state == Robot::State::STUCK){
            if(robot_.state == Robot::State::WORK){
                // work error
            }
            else if(robot_.state == Robot::State::RUN){
                // run error
            }
            else
                ROS_ERROR("error about robot state: imposible state");
        }
        else if(now_state == Robot::State::WAIT){
            // goal success, update order state
            std::pair<unsigned short int, Order> working_order_ = (VRP_Solver->working_orders).at(robot_.doing_order_id);
            if(robot_.doing_mission == "DROP_OFF"){
                working_order_.second.state = Order::State::DM;
                // send back to bridge
                send_order_state(robot_.doing_order_id, Order::State::DM, robot_id);
                // order complete update
                VRP_Solver->working_orders.erase("robot_.doing_order_id");
            }
            else if(robot_.doing_mission == "PICK_UP"){
                working_order_.second.state = Order::State::PM;
                // send back to bridge
                send_order_state(robot_.doing_order_id, Order::State::PM, robot_id);
            }
            else if(robot_.doing_mission == working_order_.second.recipient_location){
                working_order_.second.state = Order::State::DL;
                // send back to bridge
                send_order_state(robot_.doing_order_id, Order::State::DL, robot_id);
            }
            else if(robot_.doing_mission == working_order_.second.sender_location){
                working_order_.second.state = Order::State::PL;
                // send back to bridge
                send_order_state(robot_.doing_order_id, Order::State::PL, robot_id);
            }
            else if(robot_.doing_mission == "0"){
                if(robot_.goals.empty())
                    robot_.state == Robot::State::IDLE;
            }
            else{
                // imposible state
            }

            // send new robot goal
            if(robot_.goals.empty()){
                if(!send_goal(robot_, robot_id, "0", "0"))
                    ROS_ERROR("send robot %d go home FAILED", robot_id);
                robot_.doing_mission = "0";
                robot_.doing_order_id = "0";
            }
            else{
                if(send_goal(robot_, robot_id, robot_.goals.front().second, robot_.goals.front().first)){
                    robot_.doing_mission = robot_.goals.front().second;
                    robot_.doing_order_id = robot_.goals.front().first;
                    robot_.goals.pop_front();
                }
                else
                    ROS_ERROR("send robot %d to goal %s FAILED", robot_id, robot_.goals.front().second.c_str());
            }
        }
    }

    void Vehicle_Router::site_location_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        std::tuple<float, float, float> p(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        std::tuple<float, float, float, float> o(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
        short int f = msg->pose.position.z;
        ROS_INFO("get site location: %s", msg->header.frame_id.c_str());
        Site site(msg->header.frame_id.c_str(), f, p, o);
        sites.insert(std::make_pair(msg->header.frame_id.c_str(), site));
    }

    bool Vehicle_Router::send_goal(Robot& robot_, int robot_id, std::string site_id, std::string order_id){
        if(site_id == "PICK_UP"){
            // send pick up mission
            ROS_INFO("send %d pick up mission", robot_id);
            robot_.state = Robot::State::WORK;
            return true;
        }
        else if(site_id == "DROP_OFF"){
            // send drop off mission
            ROS_INFO("send %d drop off mission", robot_id);
            robot_.state = Robot::State::WORK;
            return true;
        }
        else{
            Site goal_site_ = sites[site_id];
            if(goal_site_.state == Site::State::LOCK || goal_site_.state == Site::State::MAINTAIN){
                ROS_INFO("site %s is not open right now, not send goal", site_id.c_str()); // deal with mission fail
                return false;
            }
            else{
                ROS_INFO("send %d to move to %s", robot_id, site_id.c_str());
                Interface interface_msg;
                interface_msg.mission = "move_goal";
                geometry_msgs::PoseStamped goal_pos;
                goal_pos.pose.position.x = std::get<0>(goal_site_.position);
                goal_pos.pose.position.y = std::get<1>(goal_site_.position);
                goal_pos.pose.position.z = std::get<2>(goal_site_.position);
                goal_pos.pose.orientation.x = std::get<0>(goal_site_.orientation);
                goal_pos.pose.orientation.y = std::get<1>(goal_site_.orientation);
                goal_pos.pose.orientation.z = std::get<2>(goal_site_.orientation);
                goal_pos.pose.orientation.w = std::get<3>(goal_site_.orientation);
                interface_msg.floor.data = goal_site_.floor;
                interface_msg.goal = goal_pos;
                interface_msg.robot_id.data = robot_id;
                mission_pub.publish(interface_msg);
                robot_.state = Robot::State::RUN;
                return true;
            }
        }
    }

    bool Vehicle_Router::send_order_state(std::string& order_id_, int state_, int robot_id_){
        std_msgs::Int16MultiArray msg;
        msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        msg.layout.dim[0].label = order_id_;
        msg.data.push_back(state_);
        msg.data.push_back(robot_id_);
        order_state_pub.publish(msg);
        return true;
    }
}