#ifndef ENUM_SHARED_HPP_ 
#define ENUM_SHARED_HPP_

#include <iostream>

enum HZ_Control
{
    FREQUENCY = 500,
};

enum Up_Down
{
    UP = 0,
    DOWN = 1,
    NUM_UPDOWN = 2,
};

enum Walk_Parameter
{
    SWING = 0,
    STANCE = 1,
    NUM_STATE = 2, 
    
    T_SWING = 150,
    T_STANCE = 150,
    T_TROT = T_SWING + T_STANCE,
};

enum Gait
{
    TROT,
    J_TROT,
    PRONK,
    BOUND,
    GALLOP,
    NUM_GAIT = 5,
};

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

enum DOF
{
    NUM_DOF = NUM_LEG * NUM_AXIS,
};

enum RPY
{
    ROLL,
    PITCH,
    YAW,
    NUM_RPY = 3,
};

#endif