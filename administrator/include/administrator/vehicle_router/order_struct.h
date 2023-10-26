#ifndef ORDER_STRUCT_H_
#define ORDER_STRUCT_H_

#include <string>

struct Order{
    std::string order_id;
    std::string missionType;
    short int random_password;
    std::string recipient;
    std::string recipient_location;
    std::string sender;
    std::string sender_location;
    short int priority = -1;
    enum State{WAIT, _RL, RL, _SL, SL};
    Order(
        const std::string& order_id_in,
        const std::string& missionType_in,
        const short int& random_password_in,
        const std::string& recipient_in,
        const std::string& recipient_location_in,
        const std::string& sender_in,
        const std::string& sender_location_in,
        short int priority_in) :
        order_id(order_id_in),
        missionType(missionType_in),
        random_password(random_password_in),
        recipient(recipient_in),
        recipient_location(recipient_location_in),
        sender(sender_in),
        sender_location(sender_location_in),
        priority(priority_in){};
};
#endif