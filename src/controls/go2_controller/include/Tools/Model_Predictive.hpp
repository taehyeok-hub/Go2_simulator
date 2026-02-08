#ifndef MODEL_PREDICTIVE_HPP_
#define MODEL_PREDICTIVE_HPP_

#include <vector>
#include <functional>
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <array>
#include "Eigen/Dense"
#include "Eigen/Eigenvalues"
#include "Eigen/Sparse"
#include "Eigen/Geometry"

#include "Kinematics.hpp"
#include "Gait_Generator.hpp"
#include "Enum_Shared.hpp"
#include "OsqpEigen/OsqpEigen.h"



class ModelPredictive
{
private:
    double mass;
    double mu;
    double dt; // 이산화 샘플링 시간
    const double inf = OsqpEigen::INFTY;
    Eigen::Vector3d gravity;
    Eigen::Vector3d Local_gravity;
    Eigen::Matrix3d I_body;
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    Eigen::VectorXd Body_Pos, Body_Vel, Body_Quat, Body_RPY, Body_RPY_D;
    Eigen::VectorXd Foot_FL, Foot_FR, Foot_RL, Foot_RR;
    Eigen::VectorXd Body_Ref;
    Eigen::Matrix3d R_wb, R_bw; // Rotation Matrix

    Eigen::Matrix<double, 12, 12> A_Matrix; 
    Eigen::Matrix<double, 12, 12> B_Matrix;
    Eigen::Matrix<double, 12, 1> C_Matrix;
    Eigen::Matrix<double, 12, 1> X_Matrix;
    Eigen::Matrix<double, 12, 1> X_Next;
    Eigen::VectorXd Force; // u_k

    Eigen::VectorXd Gait_Phase[NUM_LEG];
    
    Eigen::MatrixXd A_eq[4], L_eq[4], U_eq[4]; // 등식 제약 조건 2개, 부등식 제약조건 2개 
    Eigen::MatrixXd A_Cost, B_Cost;
    Eigen::MatrixXd Q_Cost, Q, R;
    Eigen::MatrixXd MPC_State; // y : 예측 상태 행렬 -> MPC_State(1) = 현재상태(Force)

    int num_of_variables;
    int num_of_constraints;

    Eigen::SparseMatrix<double> Hessian;
    Eigen::VectorXd Gradient;
    Eigen::SparseMatrix<double> LinearMatrix;
    Eigen::VectorXd LowerBound;
    Eigen::VectorXd UpperBound;
    Eigen::VectorXd QP_Solution;

    int prediction_horizon; // 계산하는 horizon
    int control_horizon;

    OsqpEigen::Solver solver;

public:
    // 생성자
    ModelPredictive();
    ~ModelPredictive();

    // 함수
    void SetBodyState(Eigen::VectorXd Body_Pos_, Eigen::VectorXd Body_Vel_, Eigen::VectorXd Body_Quat_, Eigen::VectorXd Body_RPY_, Eigen::VectorXd Body_RPY_D_);
    void SetFootState(Eigen::Vector3d Foot_FL_, Eigen::Vector3d Foot_FR_, Eigen::Vector3d Foot_RL_, Eigen::Vector3d Foot_RR_);
    void SetBodyReference(Eigen::VectorXd Body_Ref_);

    void ComputeX();
    void ComputeA();
    void ComputeB();
    void ComputeC();
    void ComputeModel();
    void UpdateXMatrix(); // test용 함수
    void SetRefGait(Eigen::VectorXd Gait_Phase[]);
    void SetCostFunction();
    void SetLinearMatrix();
    void SetEqualConstraint();
    void SetEqConstraint1();
    void SetEqConstraint2();
    void SetIneqConstraint1();
    void SetIneqConstraint2();
    void SolveQP();

    Eigen::MatrixXd GetMPCState() { return MPC_State; };
    Eigen::MatrixXd GetStateNOW() { return MPC_State.block<12,1>(12,0); };

    void SetQuatRotationMatrix(Eigen::VectorXd &quat)
    {
        /* R_bw (R^w_b) : body 좌표계에서 world 좌표계로 변환하는 행렬 해석 : (!!!World 좌표계 기준!!!)으로 Body 좌표계가 어떻게 보이는지 */
        Eigen::Quaterniond Quat(quat(3), quat(0), quat(1), quat(2));
        Quat.normalize();
        R_bw = Quat.toRotationMatrix();
        R_wb = R_bw.transpose();

        // return R_bw;
    };

    void SetRPYRotationMatrix(Eigen::VectorXd &rpy)
    {
        Eigen::Matrix3d Rx, Ry, Rz;
        Rx << 1, 0, 0, 0, cos(rpy(0)), -sin(rpy(0)), 0, sin(rpy(0)), cos(rpy(0));
        Ry << cos(rpy(1)), 0, sin(rpy(1)), 0, 1, 0, -sin(rpy(1)), 0, cos(rpy(1));
        Rz << cos(rpy(2)), -sin(rpy(2)), 0, sin(rpy(2)), cos(rpy(2)), 0, 0, 0, 1;

        R_bw = Rz * Ry * Rx;
        R_wb = R_bw.transpose();

        // return Rz*Ry*Rx;
    };

    Eigen::Matrix3d SetVecCross2Skew(Eigen::Vector3d r_vec)
    {
        Eigen::Matrix3d SkewMat;
        SkewMat << 0, -r_vec(2), r_vec(1), r_vec(2), 0, -r_vec(0), -r_vec(1), r_vec(0), 0;
        return SkewMat;
    }

    Eigen::Vector3d ErrOriCrossProduct(Eigen::Matrix3d R_act, Eigen::Matrix3d R_des)
    {
        Eigen::Vector3d cross1, cross2, cross3;
        cross1 = R_act.block<3, 1>(0, 0).cross(R_des.block<3, 1>(0, 0));
        cross2 = R_act.block<3, 1>(0, 1).cross(R_des.block<3, 1>(0, 1));
        cross3 = R_act.block<3, 1>(0, 2).cross(R_des.block<3, 1>(0, 2));

        return 0.5 * (cross1 + cross2 + cross3);
    };

    Eigen::Vector3d ErrOriso3(Eigen::Matrix3d R_act, Eigen::Matrix3d R_des)
    {
        Eigen::Matrix3d R_ee = R_des.transpose() * R_act; // Body Frame 기준
        Eigen::Matrix3d S = (R_ee - R_ee.transpose()) / 2.0;
        Eigen::Vector3d ErrVec;
        ErrVec << S(2, 1) - S(1, 2), S(0, 2) - S(2, 0), S(1, 0) - S(0, 1);

        return 0.5 * ErrVec;
    };
    
    static inline double Wrap2PI(double a)
    {
        a = std::fmod(a + M_PI, 2.0 * M_PI);
        if (a < 0.0) a += 2.0 * M_PI;
        return a - M_PI;
    }

    inline double LPF(double a1, double &filtered, double raw)
    {
        filtered = a1 * filtered + (1.0 - a1) * raw;
        return filtered;
    }

};


#endif