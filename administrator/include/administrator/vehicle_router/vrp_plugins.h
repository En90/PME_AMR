#ifndef VEHICLE_ROUTING_SOLVER_PLUGINS_H_
#define VEHICLE_ROUTING_SOLVER_PLUGINS_H_
#include <administrator/vehicle_router/vrp_base.h>

namespace vrp_plugins
{
    class Simple : public vrp_base::VehicleRoutingSolver_base
    {
        public:
            Simple(){}
            void initialize(ros::NodeHandle& nh_);
            void solve();
    };

    class Heuristic : public vrp_base::VehicleRoutingSolver_base
    {
        public:
            Heuristic(){};
            void initialize(ros::NodeHandle& nh_);
            void solve();
            void init_cost_matrix();
    };
};

#endif