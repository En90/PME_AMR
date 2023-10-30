#ifndef ROBOT_STRUCT_H_
#define ROBOT_STRUCT_H_

#include <deque>
#include <string>

struct Robot{
    unsigned short int capacity = 1;
    unsigned short int load = 0;
    enum State{RUN, STUCK, WORK, GOAL, IDLE, WAIT};
    State state = IDLE;
    std::deque<std::pair<std::string, std::string>> goals; // order_id, site_name("PICK_UP" or "DROP_OFF")
    Robot(const unsigned short int& capacity_) : capacity(capacity_){};
    unsigned short int position = 0;
    unsigned short int heading_position = 0;
    std::string doing_mission;
    std::string doing_order_id;
};

#endif
