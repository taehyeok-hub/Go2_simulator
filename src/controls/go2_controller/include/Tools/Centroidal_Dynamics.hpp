#ifndef CENTROIDAL_DYNAMICS_H
#define CENTROIDAL_DYNAMICS_H

#include <vector>
#include <functional>
#include <iostream>
#include <stdio.h>
#include <cmath>
#include "Eigen/Dense"
#include "Eigen/Eigenvalues"
#include "Eigen/Sparse"
#include "Eigen/Geometry"
#include "OsqpEigen/OsqpEigen.h"
#include "uuq_Enum.h"

class CentroidalDynamics
{
public:
    CentroidalDynamics();
    ~CentroidalDynamics();

    void Set_BodyState(Eigen::VectorXd Body_Pos_, Eigen::VectorXd Body_Vel_, Eigen::VectorXd Body_RPY_, Eigen::VectorXd Body_RPY_Dot_, Eigen::VectorXd Body_Quat_);
    void Set_Reference(Eigen::VectorXd Body_Ref_);
    void Set_FootPosition(Eigen::VectorXd Pino_F1_, Eigen::VectorXd Pino_F2_, Eigen::VectorXd Pino_H1_, Eigen::VectorXd Pino_H2_);
    void Set_LegJacobian(Eigen::MatrixXd F1_J_, Eigen::MatrixXd F2_J_, Eigen::MatrixXd H1_J_, Eigen::MatrixXd H2_J_);
    void Set_GaitPhase(int Phase[]);
    void Set_A_Matrix();
    void Set_B_Matrix();
    void Compute_CostFunction();
    void Compute_Constraint();
    void Solve_QP();

    Eigen::VectorXd GetForce(){return force;};

    Eigen::Matrix3d SkewSymmetricMatrix(const Eigen::Vector3d& vec) 
    {
        Eigen::Matrix3d skew;
        skew << 0, -vec.z(), vec.y(),
                vec.z(), 0, -vec.x(),
                -vec.y(), vec.x(), 0;
        return skew;
    }

    Eigen::Matrix3d RPYToRotationMatrix(double roll, double pitch, double yaw) 
    {
        Eigen::Matrix3d R_x, R_y, R_z;

        R_x << 1, 0, 0,
            0, cos(roll), -sin(roll),
            0, sin(roll), cos(roll);

        R_y << cos(pitch), 0, sin(pitch),
            0, 1, 0,
            -sin(pitch), 0, cos(pitch);

        R_z << cos(yaw), -sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1;

        return R_z * R_y * R_x;
    }

    Eigen::Vector3d ComputeRPYError(const Eigen::Matrix3d& R_err) 
    {
        Eigen::Vector3d E_r;
        E_r << R_err(2, 1) - R_err(1, 2),
            R_err(0, 2) - R_err(2, 0),
            R_err(1, 0) - R_err(0, 1);
        return 0.5 * E_r;
    }

    Eigen::Vector3d ComputeQuatError(const Eigen::Quaterniond& quat_ref, const Eigen::Quaterniond& quat_act)
    {
        Eigen::Quaterniond quat_error = quat_act.conjugate() * quat_ref;
        quat_error.normalize();
        return 2.0 * quat_error.vec();
    }

private:

    Eigen::VectorXd Body_Ref;
    Eigen::Quaterniond Body_Quat;
    Eigen::Vector3d Pino_F1, Pino_F2, Pino_H1, Pino_H2;
    Eigen::Vector3d Lin_Ref, Ang_Ref;
    Eigen::Vector3d Body_Pos, Body_RPY, Body_Vel, Body_RPY_Dot, Body_Err;
    Eigen::Matrix3d Body_I, Local_Body_I;
    Eigen::Matrix3d Kp_Pos, Kd_Pos, Kp_Ori, Kd_Ori;
    Eigen::Matrix3d Rz;
    Eigen::MatrixXd F1_J, F2_J, H1_J, H2_J;
    Eigen::MatrixXd A;
    Eigen::VectorXd B;
    Eigen::MatrixXd Q;
    Eigen::Vector3d Local_gravity;
    Eigen::Vector3d gravity;

    OsqpEigen::Solver solver;
    Eigen::SparseMatrix<double> Hessian;
    Eigen::SparseMatrix<double> LinearMatrix;
    Eigen::VectorXd Gradient;
    Eigen::VectorXd LowerBound;
    Eigen::VectorXd UpperBound;
    Eigen::MatrixXd Constraint_Cl_, Constraint_Cu_;
    Eigen::Matrix<double, 20, 1> Cl2, Cu2;
    Eigen::VectorXd QPSolution;
    Eigen::VectorXd force;

    int GaitPhase[NUM_LEG] = {0, 0, 0, 0};

    double mass = 144.0;
    double mu = 0.3;
};
#endif