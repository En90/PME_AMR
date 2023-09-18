#include <administrator/vehicle_router/vehicle_router.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "vehicle_router");
  administrator::Vehicle_Router node;
  ros::spin();
  return 0;
}
