#ifndef VEHICLE_ROUTER_H_
#define VEHICLE_ROUTER_H_

#include <ros/ros.h>
#include <queue>
#include <unordered_map>
#include <string>
#include <vector>
#include <std_msgs/String.h>
#include "administrator/order_msgs.h"

namespace administrator{
    class Vehicle_Router{
        private:
            struct Order{
                std::string order_id;
                std::string missionType;
                std::string random_password;
                std::string recipient;
                std::string recipient_location;
                std::string sender;
                std::string sender_location;
                short int priority = -1;
                enum State{WAIT, _RL, RL, _SL, SL};
                Order(
                    const std::string& order_id_in,
                    const std::string& missionType_in,
                    const std::string& random_password_in,
                    const std::string& recipient_in,
                    const std::string& recipient_location_in,
                    const std::string& sender_in,
                    const std::string& sender_location_in,
                    short int priority_in) :
                    order_id(order_id_in),
                    missionType(missionType_in),
                    random_password(random_password_in),
                    recipient(recipient_in),
                    recipient_location(recipient_location_in),
                    sender(sender_in),
                    sender_location(sender_location_in),
                    priority(priority_in){};
            };
            struct Order_Comp{
                bool operator()(Order a, Order b){
                    return a.priority > b.priority;
                }
            };
            struct Robot{
                unsigned short int capacity = 1;
                enum State{IDLE, WAIT, RUN, STUCK};
                State state = IDLE;
                std::queue<std::pair<std::string, std::string>> goals; // order_id, goal_id
            };
            class VRP_Solver_temp{
                private:
                    unsigned short int robot_num = 1;
                    void solve(std::vector<Robot>& robots, std::priority_queue<Order, std::vector<Order>, Order_Comp>& confirmed_waiting_q, std::unordered_map<std::string, std::pair<unsigned short int, Order>>& working_orders);

                public:
                    VRP_Solver_temp(const unsigned short int& set_robot_num);
            };
            ros::NodeHandle nh;
            std::priority_queue<Order, std::vector<Order>, Order_Comp> confirmed_waiting_q;
            std::unordered_map<std::string, std::pair<unsigned short int, Order>> working_orders; // key[order_id]: value[<robot_id, order>]
            const std::size_t robot_num_ = 1; // get from ros param
            std::vector<Robot> robots = std::vector<Robot>(robot_num_);
            ros::Subscriber order_sub;
            ros::Publisher order_state_pub;
            void order_callback(const order_msgs::ConstPtr& msg);
            VRP_Solver_temp vrp_solver = VRP_Solver_temp(robot_num_); // can be load from plugin

        public:
            Vehicle_Router();
    };
};

#endif