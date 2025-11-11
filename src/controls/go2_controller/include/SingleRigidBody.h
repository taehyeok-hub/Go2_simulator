#include <iostream>
#include <array>
#include <cmath>

#include "Eigen/Dense"
#include "Eigen/Geometry"

class SingleRigidBody // 선언부
{   // robot 사양 : M_몸통 : 6.921kg, <box size="0.3762 0.0935 0.114" />   
private:
    double M; // 로봇 몸통의 질량
    Eigen::Matrix3d I_body; // 몸통의 관성 텐서
    Eigen::Vector3d g_world; // 월드좌표계에서의 중력가속도  

    Eigen::Vector3d p_com_w;
    std::array<Eigen::Vector3d, 4> p_leg_w; // fl, fr, rl, rr 순
    std::array<Eigen::Vector3d, 4> r_leg_b; // (r)을 의미합니다.

    Eigen::Matirx3d R_world_to_body; // world -> body 변환행렬

    

    // 각 다리의 지면 반발력을 담는 힘 벡터
    Eigen::Vector3d f_FL;
    Eigen::Vector3d f_FR;
    Eigen::Vector3d f_RL;
    Eigen::Vector3d f_RR;


    // AF = b  ---> A : 6x12 행렬, F : 12x1 행렬, b : 6x1 행렬
    Eigen::Matrix<double, 6, 12> A_Matirx_b {Eigen::Matrix<double,6,12>::Zero()}
    Eigen::VectorXd F  {Eigen::VectorXd::Zero(12)};
    Eigen::VectorXd b_Vector_b  {Eigen::VectorXd::Zero(6)};


public:
    SingleRigidBody(); // 생성자
    void Update_A_Matrix()
    void Solve_SRBM_Force()
    void ComputeDesiredWrench()
    
    

    Eigen::Matirx3d MakeCross2Skew(const Eigen::Vector3d& p) // 선언만 진행함.
    
    
    

}