#ifndef KINEMATICS_HPP_ 
#define KINEMATICS_HPP_

#include <iostream>
#include <array>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "Enum_Shared.hpp"

class Kinematics
{
private:
    // Constants ------------------------------------------------------------------------------------------------------------------------------------------------------------------

    const double base_to_hip_x = 0.1934; // 0.19275
    const double base_to_hip_y = 0.0465; // 0.145
    const double hip_to_thigh_y = 0.0955; // 0.085
    const double thigh_to_calf_z = -0.213; // 0.215
    const double calf_to_foot_z = -0.213; // 0.215

    // 회전 축
    const Eigen::Vector3d hip_axis = {1, 0, 0};
    const Eigen::Vector3d thigh_axis = {0, 1, 0};
    const Eigen::Vector3d calf_axis = {0, 1, 0};

    // Variables ------------------------------------------------------------------------------------------------------------------------------------------------------------------

    std::array<Eigen::Vector3d, 4> EE_Pose; 
    
    // Eigen::Vector3d EE_Pose_FL  {Eigen::Vector3d::Zero()};
    // Eigen::Vector3d EE_Pose_FR  {Eigen::Vector3d::Zero()};
    // Eigen::Vector3d EE_Pose_RL  {Eigen::Vector3d::Zero()};
    // Eigen::Vector3d EE_Pose_RR  {Eigen::Vector3d::Zero()};

    Eigen::VectorXd IK_results  {Eigen::VectorXd::Zero(12)};

    Eigen::Matrix<double, 6, 3> J[NUM_LEG];
    Eigen::Matrix<double, 6, 12> J_Matrix = Eigen::Matrix<double, 6, 12>::Zero();

    // Sub Functions ------------------------------------------------------------------------------------------------------------------------------------------------------------------
    
    Eigen::Matrix4d CreateRmatrix(double angle, const Eigen::Vector3d& axis);
    Eigen::Matrix4d CreateTmatrix(double x, double y, double z);
    Eigen::Matrix4d Transformation_Matrix(double theta, double d, double a, double alpha);
    Eigen::Vector3d Calculate_Atan2(Eigen::Vector3d& p_thigh, double a2, double a3);
    Eigen::Vector3d Calculate_V(Eigen::Vector3d& ee, Eigen::Vector3d& bb);

public:
    // Constructor
    Kinematics();
    ~Kinematics();

    // Main Function
    void Forward_Kinematics(const Eigen::VectorXd& q, const Eigen::VectorXd& dq);
    void Inverse_Kinematics();
    void Jacobian(const Eigen::VectorXd& q);

    // Getter Function
    std::array<Eigen::Vector3d, 4> Get_EE_Pose() const { return EE_Pose; }
    Eigen::Matrix<double, 6, 12> Get_Jacobian() const { return J_Matrix; }

};

#endif