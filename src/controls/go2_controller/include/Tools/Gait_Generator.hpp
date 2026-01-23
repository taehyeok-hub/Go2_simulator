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


class Gait_Generator 
{
private:
    // Constant



    // Variables
    Eigen::VectorXd stance[NUM_LEG];
    Eigen::VectorXd Target_State;
    
    Eigen::MatrixXd Trot_Pattern;

public:
    Gait_Generator();
    ~Gait_Generator();

    void Gait_Update();
    void Trot_Gait_Generator(double switch_time = 0.3);
    Eigen::VectorXd Get_ReferenceGait(int leg) { return Gait_Timing[leg];}

    Gait gaitmode; // enum 객체 생성

    Eigen::VectorXd Gait_Timing[NUM_LEG];
    int temp[NUM_LEG];

    bool Init_Trot = true; // 첫번째 트롯이냐?
    int Stand_Time = 0;
    




    // Get 함수
    Eigen::Vector4d Get_Trot_Gait() {return Target_State; };


};

#endif