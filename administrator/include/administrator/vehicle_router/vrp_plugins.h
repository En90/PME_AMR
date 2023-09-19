#ifndef VEHICLE_ROUTING_SOLVER_PLUGINS_H_
#define VEHICLE_ROUTING_SOLVER_PLUGINS_H_
#include <administrator/vehicle_router/vrp_base.h>

namespace vrp_plugins
{
    class Simple : public vrp_base::VehicleRoutingSolver_base
    {
        public:
            Simple(){}
            void initialize(const unsigned short int& robot_num_);
            void solve();
    };
};

#endif