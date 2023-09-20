#include "administrator/vehicle_router/vehicle_router.h"

namespace administrator{
    Vehicle_Router::Vehicle_Router() : nh("~"){
        //Load param
        
        //Init Sub and Pub
        order_sub = nh.subscribe("/confirmed_order", 10, &Vehicle_Router::order_callback, this);
        robot_sub = nh.subscribe("/robot_state", 10, &Vehicle_Router::robot_callback, this);
        order_state_pub = nh.advertise<std_msgs::String>("/order_state", 1);

        //Load plugin
        std::string plugin_("vrp_plugins::Simple"); // can be load as ros param
        try{
            pluginlib::ClassLoader<vrp_base::VehicleRoutingSolver_base> vrp_loader("administrator", "vrp_base::VehicleRoutingSolver_base");
            VRP_Solver = vrp_loader.createInstance(plugin_);
            VRP_Solver->initialize(robot_num, robot_capacity);
        }
        catch(pluginlib::PluginlibException& ex){
            ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        }
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
}