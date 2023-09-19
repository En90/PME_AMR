#ifndef SITE_STRUCT_H_
#define SITE_STRUCT_H_

#include <string>
#include <tuple>

struct Site{
    std::string name;
    short int floor;
    std::tuple<float, float, float> position;
    std::tuple<float, float, float, float> orientation;
    enum State{LOCK, IDLE, MAINTAIN, DOCKING, OCCUPY};
    State state;
};

#endif