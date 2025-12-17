#ifndef CENTROIDAL_DYNAMICS_HPP_ 
#define CENTROIDAL_DYNAMICS_HPP_

#include <iostream>
#include <array>
#include <Eigen/Dense>
#include <Eigen/Geometry>


#include "Kinematics.hpp"
#include "Enum_Shared.hpp"
#include "OsqpEigen/OsqpEigen.h"

class Centroidal_Dynamics
{
private:
    // Constant ------------------------------------------------------------------------------------------------------------------------------------------------------------------
    double mass;
    double mu;
    Eigen::Vector3d gravity;
    Eigen::Matrix3d I_body;

    // Variable ------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // 1. 로봇 발끝 위치 (바로 저장하는 값) - FK 저장 (body 좌표계)
    Eigen::Vector3d EE_Pose_FL_Body, EE_Pose_FR_Body, EE_Pose_RL_Body, EE_Pose_RR_Body;

    // 2. 로봇 몸통 위치/속도/오일러각/각속도/쿼터니언 - World 좌표계
    Eigen::VectorXd COM_Pose, COM_Vel, COM_Quat, COM_rpy, COM_rpy_dot;
    
    // 3. 로봇 몸통의 desired / actual / error
    Eigen::Vector3d des_Pos, des_Vel, des_Ori, des_Ori_dot, act_Ori_dot;
    Eigen::Vector3d Err_Pos, Err_Ori;

    // 4. Rotation Matrix
    Eigen::Matrix3d R_wb, R_bw;

    // 5. Kp , Kd gain
    Eigen::Matrix3d Kp_Pos, Kd_Pos, Kp_Ori, Kd_Ori;

    // 6. QP Solve
    const int num_of_variables = NUM_LEG * NUM_AXIS; // = 본인 변수 크기
    const int num_of_constraints = 5 * NUM_LEG; // = 본인 제약조건 크기

    // 7. Matrix 저장
    Eigen::MatrixXd A_Matrix = Eigen::MatrixXd::Zero(6,12);
    Eigen::VectorXd B_Vector = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd F_Vector = Eigen::VectorXd::Zero(12);
    
    Eigen::SparseMatrix<double> Hessian;
    Eigen::VectorXd Gradient;
    Eigen::SparseMatrix<double> LinearMatrix;
    Eigen::VectorXd LowerBound;
    Eigen::VectorXd UpperBound;
    Eigen::VectorXd QP_Solution;

    // 7. Class 객체 선언
    OsqpEigen::Solver solver;


    // Function ------------------------------------------------------------------------------------------------------------------------------------------------------------------

    void Solve_QP();
    void Set_RobotState(Eigen::VectorXd COM_Pose_, Eigen::VectorXd COM_Vel_, Eigen::VectorXd COM_Quat_, Eigen::VectorXd COM_rpy_, Eigen::VectorXd COM_rpy_dot_);
    void Set_FootPosition(std::array<Eigen::Vector3d, 4> EE_Pose);
    void Set_CostFunction();
    void Set_Reference();
    void Compute_A_Matrix();
    void Compute_B_Vector(Eigen::Vector3d des_Pos_, Eigen::Matrix3d des_Ori_);

    void Set_QuatRotationMatrix(Eigen::VectorXd &quat)
    {
        /* R_bw (R^w_b) : body 좌표계에서 world 좌표계로 변환하는 행렬 해석 : (!!!World 좌표계 기준!!!)으로 Body 좌표계가 어떻게 보이는지 */
        Eigen::Quaterniond Quat(quat(3), quat(0), quat(1), quat(2));
        Quat.normalize();
        R_bw = Quat.toRotationMatrix();
        R_wb = R_bw.transpose();

        // return R_bw;
    };

    void Set_RPYRotationMatrix(Eigen::VectorXd &rpy)
    {
        Eigen::Matrix3d Rx, Ry, Rz;
        Rx << 1, 0, 0,  0, cos(rpy(0)), -sin(rpy(0)),  0, sin(rpy(0)), cos(rpy(0));
        Ry << cos(rpy(1)), 0, sin(rpy(1)),  0, 1, 0,  -sin(rpy(1)), 0, cos(rpy(1));
        Rz << cos(rpy(2)), -sin(rpy(2)), 0,  sin(rpy(2)), cos(rpy(2)), 0,  0, 0, 1;  
        
        R_bw = Rz*Ry*Rx;
        R_wb = R_bw.transpose();

        // return Rz*Ry*Rx;
    };

    Eigen::Matrix3d Set_VecCross2Skew(Eigen::Vector3d vec)
    {
        // (p - pcom)X 을 Skew Symmetric으로 바꾸는 함수
        Eigen::Matrix3d SkewMat;
        SkewMat << 0, -vec(2), vec(1),  vec(2), 0, vec(0),  -vec(1), vec(0), 0;

        return SkewMat;
    };

    Eigen::Vector3d ErrOri_CrossProduct(Eigen::Matrix3d R_act, Eigen::Matrix3d R_des)
    {
        Eigen::Vector3d cross1, cross2, cross3;
        cross1 = R_act.block<3,1>(0,0).cross(R_des.block<3,1>(0,0));
        cross2 = R_act.block<3,1>(0,1).cross(R_des.block<3,1>(0,1));
        cross3 = R_act.block<3,1>(0,2).cross(R_des.block<3,1>(0,2));

        return cross1+cross2+cross3;
    };

    Eigen::Matrix3d ErrOri_so3(Eigen::Matrix3d R_act, Eigen::Matrix3d R_des)
    {
        Eigen::Matrix3d R_ee = R_des.transpose() * R_act;
        Eigen::Matrix3d S = (R_ee - R_ee.transpose()) / 2.0;
        Eigen::Vector3d ErrVec;
        ErrVec << S(2,1) - S(1,2), S(0,2) - S(2,0), S(1,0) - S(0,1);

        return 0.5 * ErrVec;
    };

public:
    Centroidal_Dynamics();
    ~Centroidal_Dynamics();

    // get 함수
    Eigen::VectorXd Get_Force() {return F_Vector; };
  

};






#endif