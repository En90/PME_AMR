#include <pluginlib/class_list_macros.h>
#include <administrator/vehicle_router/vrp_base.h>
#include <administrator/vehicle_router/vrp_plugins.h>

PLUGINLIB_EXPORT_CLASS(vrp_plugins::Simple, vrp_base::VehicleRoutingSolver_base)
PLUGINLIB_EXPORT_CLASS(vrp_plugins::Heuristic, vrp_base::VehicleRoutingSolver_base)

namespace vrp_plugins
{
    void Simple::initialize(ros::NodeHandle& nh_){
        nh = nh_;
        init_robots();
        init_cost_matrix();
    }

    void Simple::solve(){
        for(std::size_t robot_id = 0; robot_id < robots.size(); robot_id++){
            bool condition = robots[robot_id].state == Robot::IDLE && robots[robot_id].goals.empty() && !waiting_order.empty();
            if(condition){
                Order order = waiting_order.top();
                waiting_order.pop();
                robots[robot_id].goals.emplace_back(std::make_pair(order.order_id, order.sender_location));
                robots[robot_id].goals.emplace_back(std::make_pair(order.order_id, "PICK_UP"));
                robots[robot_id].goals.emplace_back(std::make_pair(order.order_id, order.recipient_location));
                robots[robot_id].goals.emplace_back(std::make_pair(order.order_id, "DROP_OFF"));
                auto working_order = std::make_pair(robot_id, order);
                auto retPair = working_orders.insert(std::pair<std::string, std::pair<unsigned short int, Order>>(order.order_id, working_order));
                // if (retPair.second == true) ROS_INFO("Insert Successfully\n");
            }
            else
                continue;
        }
    }

    void Heuristic::initialize(ros::NodeHandle& nh_){
        nh = nh_;
        init_robots();
        init_cost_matrix();
    }

    void Heuristic::solve(){

    }

    void Heuristic::init_cost_matrix(){

    }
};