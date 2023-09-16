#include <administrator/vehicle_router/vehicle_router.h>

namespace administrator{
    Vehicle_Router::Vehicle_Router() : nh("~"){
        //Load param
        
        //Init Sub and Pub
        //order_sub = nh.subscribe("/confirmed_order", 10, &order_callback, this);
        order_state_pub = nh.advertise<std_msgs::String>("/order_state", 1);
    }

    // void order_callback(){

    // }
}