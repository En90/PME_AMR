#ifndef VEHICLE_ROUTING_SOLVER_BASE_H_
#define VEHICLE_ROUTING_SOLVER_BASE_H_
#include <vector>
#include <string>

namespace vrp_base{
    class VehicleRoutingSolver_base{
        public:
            virtual void initialize();
            virtual std::vector<std::vector<std::string>> plan();
            virtual ~VehicleRoutingSolver_base(){}

        protected:
            VehicleRoutingSolver_base(){}
    };
};

#endif