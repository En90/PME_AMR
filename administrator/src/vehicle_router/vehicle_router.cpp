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
        order_state_pub = nh.advertise<std_msgs::String>("/order_state", 1);
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
    }

    void Vehicle_Router::robot_callback(const std_msgs::Float32MultiArray& msg){
        int robot_id = msg.data[0];
        VRP_Solver->robots[robot_id].state = static_cast<Robot::State>(msg.data[1]);
    }

    void Vehicle_Router::site_location_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        std::tuple<float, float, float> p(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        std::tuple<float, float, float, float> o(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
        short int f = msg->pose.position.z;
        ROS_INFO("get site location: %s", msg->header.frame_id.c_str());
        Site site(msg->header.frame_id.c_str(), f, p, o);
        sites.insert(std::make_pair(msg->header.frame_id.c_str(), site));
    }
}