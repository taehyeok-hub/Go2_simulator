#ifndef ENUM_SHARED_HPP_ 
#define ENUM_SHARED_HPP_

#include <iostream>

enum HZ_Control
{
    FREQUENCY = 500,
};

enum Walk_Parameter
{
    STANCE = 0,
    SWING = 1,
    
    TROT_T = 500,

    SWING_TROT = 120,
    STANCE_TROT = 120,
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