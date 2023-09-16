#include <pluginlib/class_list_macros.h>
#include <administrator/vehicle_router/vrp_base.h>
#include <administrator/vehicle_router/vrp_plugins.h>

PLUGINLIB_EXPORT_CLASS(vrp_plugins::Simple, vrp_base::VehicleRoutingSolver_base)