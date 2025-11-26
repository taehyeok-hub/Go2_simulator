#ifndef ENUM_SHARED_HPP_ 
#define ENUM_SHARED_HPP_

#include <iostream>

enum Axis
{
    X,
    Y,
    Z,
    NUM_AXIS = 3,
};

enum Joint
{
    HR,
    HP,
    KP,
    NUM_JOINT = 3,
};

enum Leg_Num
{
    FL,
    FR,
    RL,
    RR,
    NUM_LEG = 4,
};

#endif