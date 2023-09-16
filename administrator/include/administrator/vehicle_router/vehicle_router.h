#ifndef VEHICLE_ROUTER_H_
#define VEHICLE_ROUTER_H_

#include <ros/ros.h>
#include <queue>
#include <string>
#include <std_msgs/String.h>

namespace administrator{
    class Vehicle_Router{
        private:
            ros::NodeHandle nh;
            //ros::Subscriber order_sub;
            ros::Publisher order_state_pub;
            //void order_callback();
        public:
            Vehicle_Router();
    };
};

#endif