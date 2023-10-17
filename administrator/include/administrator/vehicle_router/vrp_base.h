#ifndef VEHICLE_ROUTING_SOLVER_BASE_H_
#define VEHICLE_ROUTING_SOLVER_BASE_H_

#include <vector>
#include <string>
#include <queue>
#include <unordered_map>
#include <ros/ros.h>
#include "administrator/vehicle_router/order_struct.h"
#include "administrator/vehicle_router/robot_struct.h"
#include "administrator/vehicle_router/site_struct.h"

namespace vrp_base{
    class VehicleRoutingSolver_base{
        private:
            // not inherient
        public:
            struct Order_Comp{
                bool operator()(Order a, Order b){
                    return a.priority > b.priority;
                }
            };
            std::priority_queue<Order, std::vector<Order>, Order_Comp> waiting_order;
            std::unordered_map<std::string, std::pair<unsigned short int, Order>> working_orders; // key[order_id]: value[<robot_id, order>]
            std::vector<Robot> robots;
            std::vector<std::vector<unsigned int>> cost_matrix;
            std::unordered_map<std::string, unsigned int> site_map;
            ros::NodeHandle nh;
            int robot_num = 1; 
            virtual void initialize(ros::NodeHandle& nh_){};
            virtual void solve(){};
            virtual void init_cost_matrix(){
                //init site_map
                //init cost matrix
            };
            virtual void init_robots(){
                int robot_capacity = 1;
                // load param 
                if(nh.getParam("vehicle_routing_problem/robot_number", robot_num) == false){
                    ROS_WARN("Can not get param: vehicle_routing_problem/robot_number");
                    ROS_WARN("Use default value: 1");
                }
                if(nh.getParam("vehicle_routing_problem/robot_capacity", robot_capacity) == false){
                    ROS_WARN("Can not get param: vehicle_routing_problem/robot_capacity");
                    ROS_WARN("Use default value: 1");
                }
                // init robots
                robots.clear();
                for(int i = 0; i < robot_num; i++){
                    Robot robot(robot_capacity);
                    robots.emplace_back(robot);
                }
            };
            virtual void add_order(Order& order_){
                waiting_order.emplace(order_);
            };
            virtual ~VehicleRoutingSolver_base(){}

        protected:
            VehicleRoutingSolver_base(){}
    };
};

#endif