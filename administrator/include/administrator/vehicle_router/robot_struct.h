#ifndef ROBOT_STRUCT_H_
#define ROBOT_STRUCT_H_

#include <queue>
#include <string>

struct Robot{
    unsigned short int capacity = 1;
    enum State{IDLE, WAIT, RUN, STUCK};
    State state = IDLE;
    std::queue<std::pair<std::string, std::string>> goals; // order_id, goal_id
    Robot(const unsigned short int& capacity_) : capacity(capacity_){};
};

#endif
