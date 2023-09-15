#include "ros/ros.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "vehicle_router");
  ros::NodeHandle nh;
  ros::spin();
  return 0;
}
