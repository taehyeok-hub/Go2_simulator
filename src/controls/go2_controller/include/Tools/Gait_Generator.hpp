#ifndef GAIT_GENERATOR_HPP_
#define GAIT_GENERATOR_HPP_

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <functional>
#include <vector>
#include <array>

#include "Enum_Shared.hpp"
#include "Kinematics.hpp"
#include "Pinocchio_Interface.hpp"

/* 
Gait Generation: 보행 생성 (전체적인 걷기 전략)

Gait Control: 보행 제어

Gait Transition: 보행 전환 (걷다가 뛰기로 바꿀 때)
*/


// struct GaitPattern
// {
//     Eigen::RowVector4d State1;
//     Eigen::RowVector4d State2;
//     Eigen::RowVector4d State3;
// };

class Gait_Generator 
{
private:
    // Constant



    // Variables
    Eigen::Vector3d stance[NUM_LEG];
    Eigen::Vector4d Target_State;
    Eigen::Vector4d State1, State2, State3, State4;
    
    int Switch_Time = 0;
    int Gait_Switch = 0;

    // GaitPattern Gait;
    Eigen::MatrixXd Trot_Pattern;

public:
    Gait_Generator();
    ~Gait_Generator();

    void Trot_Gait_Generator(double switch_time = 0.3);

    void Gait_Update();

    Eigen::VectorXd GetRefGait(int leg) { return Gait_Timing[leg];}

    Gait gaitmode;

    Eigen::VectorXd Gait_Timing[NUM_LEG];
    int temp[NUM_LEG];

    bool Init_Trot = false;
    


    




    // Get 함수
    Eigen::Vector4d Get_Trot_Gait() {return Target_State; };


};

#endif