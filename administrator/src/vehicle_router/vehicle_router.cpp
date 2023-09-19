#include "administrator/vehicle_router/vehicle_router.h"

namespace administrator{
    Vehicle_Router::Vehicle_Router() : nh("~"){
        //Load param
        
        //Init Sub and Pub
        order_sub = nh.subscribe("/confirmed_order", 10, &Vehicle_Router::order_callback, this);
        order_state_pub = nh.advertise<std_msgs::String>("/order_state", 1);

        //Load plugin
        std::string plugin_("vrp_plugins::Simple"); // can be load as ros param
        try{
            pluginlib::ClassLoader<vrp_base::VehicleRoutingSolver_base> vrp_loader("administrator", "vrp_base::VehicleRoutingSolver_base");
            VRP_Solver = vrp_loader.createInstance(plugin_);
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
        confirmed_waiting_q.emplace(order);
        ROS_INFO("%ld orders in waiting.", confirmed_waiting_q.size());
    }
}