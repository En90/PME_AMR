#ifndef VEHICLE_ROUTER_H_
#define VEHICLE_ROUTER_H_

#include <ros/ros.h>
#include <queue>
#include <unordered_map>
#include <string>
#include <vector>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
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
            const std::size_t robot_num = 1; // get from ros param
            const unsigned short int robot_capacity = 1; // get from ros param
            ros::Subscriber order_sub;
            ros::Subscriber robot_sub;
            ros::Publisher order_state_pub;
            void order_callback(const order_msgs::ConstPtr& msg);
            void robot_callback(const std_msgs::Float32MultiArray& msg);
            boost::shared_ptr<vrp_base::VehicleRoutingSolver_base> VRP_Solver;

        public:
            Vehicle_Router();
    };
};

#endif