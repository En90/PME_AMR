#ifndef VEHICLE_ROUTER_H_
#define VEHICLE_ROUTER_H_

#include <ros/ros.h>
#include <queue>
#include <unordered_map>
#include <string>
#include <vector>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
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
            // variable
            ros::NodeHandle nh;
            ros::Subscriber order_sub;
            ros::Subscriber robot_sub;
            ros::Subscriber site_location_sub;
            ros::Publisher order_state_pub;
            boost::shared_ptr<vrp_base::VehicleRoutingSolver_base> VRP_Solver;
            std::vector<Robot> robots;
            const std::size_t robot_num = 1; // get from ros param
            const unsigned short int robot_capacity = 1; // get from ros param
            // callback
            void order_callback(const order_msgs::ConstPtr& msg);
            void robot_callback(const std_msgs::Float32MultiArray& msg); // robot_id robot_state robot_position 
            void site_location_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
            // container
            std::unordered_map<std::string, Site> sites;
            // timer
            void mission_management(){};

        public:
            Vehicle_Router();
            ~Vehicle_Router();
    };
};

#endif