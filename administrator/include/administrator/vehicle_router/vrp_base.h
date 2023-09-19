#ifndef VEHICLE_ROUTING_SOLVER_BASE_H_
#define VEHICLE_ROUTING_SOLVER_BASE_H_

#include <vector>
#include <string>
#include <queue>
#include <unordered_map>
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
            virtual void initialize(const unsigned short int& robot_num_){};
            virtual void solve(){};
            virtual ~VehicleRoutingSolver_base(){}

        protected:
            VehicleRoutingSolver_base(){}
    };
};

#endif