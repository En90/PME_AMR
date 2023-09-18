#include <administrator/vehicle_router/vehicle_router.h>

namespace administrator{
    Vehicle_Router::Vehicle_Router() : nh("~"){
        //Load param
        
        //Init Sub and Pub
        order_sub = nh.subscribe("/confirmed_order", 10, &Vehicle_Router::order_callback, this);
        order_state_pub = nh.advertise<std_msgs::String>("/order_state", 1);
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
        confirmed_waiting_q.emplace(order);
        ROS_INFO("%ld orders in waiting.", confirmed_waiting_q.size());
    }

    Vehicle_Router::VRP_Solver_temp::VRP_Solver_temp(const unsigned short int& set_robot_num){
        robot_num = set_robot_num;
    }

    void Vehicle_Router::VRP_Solver_temp::solve(std::vector<Robot>& robots, std::priority_queue<Order, std::vector<Order>, Order_Comp>& confirmed_waiting_q, std::unordered_map<std::string, std::pair<unsigned short int, Order>>& working_orders){
        for(std::size_t robot_id = 0; robot_id < robots.size(); robot_id++){
            bool condition = robots[robot_id].state == Robot::IDLE && robots[robot_id].goals.empty() && !confirmed_waiting_q.empty();
            if(condition){
                Order order = confirmed_waiting_q.top();
                confirmed_waiting_q.pop();
                robots[robot_id].goals.emplace(std::make_pair(order.order_id, order.sender_location));
                robots[robot_id].goals.emplace(std::make_pair(order.order_id, order.recipient_location));
                auto working_order = std::make_pair(robot_id, order);
                auto retPair = working_orders.insert(std::pair<std::string, std::pair<unsigned short int, Order>>(order.order_id, working_order));
                if (retPair.second == true) ROS_INFO("Insert Successfully\n");
            }
            else
                continue;
        }
    }
}