#ifndef VEHICLE_ROUTER_H_
#define VEHICLE_ROUTER_H_

#include <ros/ros.h>
#include <queue>
#include <unordered_map>
#include <string>
#include <vector>
#include <std_msgs/String.h>
#include <pluginlib/class_loader.h>
#include "administrator/order_msgs.h"
#include "administrator/vehicle_router/order_struct.h"
#include "administrator/vehicle_router/robot_struct.h"
#include "administrator/vehicle_router/site_struct.h"
#include "administrator/vehicle_router/vrp_base.h"

namespace administrator{
    class Vehicle_Router{
        private:
            struct Order_Comp{
                bool operator()(Order a, Order b){
                    return a.priority > b.priority;
                }
            };
            ros::NodeHandle nh;
            std::priority_queue<Order, std::vector<Order>, Order_Comp> confirmed_waiting_q;
            std::unordered_map<std::string, std::pair<unsigned short int, Order>> working_orders; // key[order_id]: value[<robot_id, order>]
            const std::size_t robot_num_ = 1; // get from ros param
            std::vector<Robot> robots = std::vector<Robot>(robot_num_);
            ros::Subscriber order_sub;
            ros::Publisher order_state_pub;
            void order_callback(const order_msgs::ConstPtr& msg);
            void init_robots();
            boost::shared_ptr<vrp_base::VehicleRoutingSolver_base> VRP_Solver;

        public:
            Vehicle_Router();
    };
};

#endif