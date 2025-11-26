#ifndef SINGLERIGIDBODY_HPP_  
#define SINGLERIGIDBODY_HPP_

#include <iostream>
#include <array>
#include <cmath>

#include "Eigen/Dense"
#include "Eigen/Eigenvalues"
#include "Eigen/Sparse"
#include "Eigen/Geometry"
#include "OsqpEigen/OsqpEigen.h"
#include "Enum_Shared.hpp"


class SingleRigidBody
{
private:
    // 1. 상수 (Constants) ------------------------------------------------------

    double M;
    double mu;
    Eigen::Matrix3d I_body;  // 바디 좌표계 기준 관성 텐서
    Eigen::Vector3d gravity; // 월드 좌표계 기준 중력 벡터
    Eigen::Vector3d gravity_body;

    double alpha;

    // 2. "원본 상태" 변수 (State Variables) ------------------------------------------------------------------------------------------------------------------------------------------------------------------

    Eigen::Vector3d p_com_world;
    Eigen::Vector3d v_com_world;
    Eigen::Quaterniond quat_world;
    Eigen::Vector3d rpy_world;
    Eigen::Vector3d omega_world;     // CoM 각속도 (바디 기준, IMU 센서 값)
    Eigen::Matrix3d R_world_to_body; // 자세 (월드 -> 바디)
    Eigen::Matrix3d R_body_to_world; // 자세 (바디 -> 월드)

    std::array<Eigen::Vector3d, 4> p_foot_world; // fl, fr, rl, rr 순

    const int num_of_variables = 12; // 본인 변수 크기 : (발 4개 x 3축)
    const int num_of_constraints = 5; // 본인 제약조건 크기 : 나오는 부등식 개수  

    // 3. 계산용 내부 변수 (Internal Variable) ------------------------------------------------------------------------------------------------------------------------------------------------------------------

    // AF = b
    Eigen::Matrix<double, 6, 12> A_mat;
    Eigen::VectorXd b_vec; // (6x1)
    Eigen::VectorXd F_world; // (12x1)

    // QP
    Eigen::SparseMatrix<double> Hessian;
    Eigen::VectorXd Gradient;
    Eigen::SparseMatrix<double> LinearMatrix;
    Eigen::VectorXd LowerBound;
    Eigen::VectorXd UpperBound;
    
    Eigen::VectorXd QP_Solution;
    
    Eigen::Matrix3d Kp_Pos, Kd_Pos, Kp_Ori, Kd_Ori;
    Eigen::Matrix<double, 12, 12> Q; // 내부 변수

    // 4. 내부 헬퍼 함수 ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    Eigen::Matrix3d MakeCross2Skew(const Eigen::Vector3d &p);
    Eigen::Vector3d MakeMatrix2Skew(const Eigen::Matrix3d &R1, const Eigen::Matrix3d &R2);
    Eigen::Matrix3d RPYRotationMatrix(double roll, double pitch, double yaw);

    // 5. 사용 클래스 ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    OsqpEigen::Solver solver;

public:
    SingleRigidBody(); // 생성자
    ~SingleRigidBody(); // 소멸자

    // 외부(go2_controller)에서 이 함수들을 호출해 상태를 주입함.
    void SetRobotState(const Eigen::VectorXd &p, const Eigen::VectorXd &p_dot, const Eigen::VectorXd &quat, const Eigen::VectorXd &rpy, const Eigen::VectorXd &rpy_dot);

    void SetFootPosition(const std::array<Eigen::Vector3d, 4> &feet_pos, Eigen::VectorXd &p_com);

    void Update_A_Matrix();
    void Compute_b_Vector(const Eigen::Vector3d &p_des_world, const Eigen::Matrix3d &R_des_body,
                          double Kp_pos, double Kd_pos, double Kp_ori, double Kd_ori);
                          
    void Compute_LinearMatrix(double friction_mu = 0.7);
    void Compute_Constraint(double f_z_max = 200);
    void Solve_Force();
    void Solve_QP();

    // 외부에서 계산된 힘 결과를 가져감
    Eigen::VectorXd Get_Force() { return F_world; }
};

#endif