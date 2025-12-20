#ifndef PINOCCHIOINTERFACE_H_
#define PINOCCHIOINTERFACE_H_

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/math/rpy.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/centroidal-derivatives.hpp>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <iostream>
#include <string>
#include <vector>

#include "Enum_Shared.hpp"

class PinocchioInterface
{
public:
    static constexpr int kTaskDim = 3; // (x,y,z)

    using TaskVector   = Eigen::Matrix<double, kTaskDim, 1>;          // 3x1
    using TaskMatrix   = Eigen::Matrix<double, kTaskDim, kTaskDim>;   // 3x3
    using JointVector  = Eigen::Matrix<double, NUM_JOINT, 1>;         // NUM_JOINT x 1
    using TaskJacobian = Eigen::Matrix<double, kTaskDim, NUM_JOINT>;  // 3 x NUM_JOINT
    using JointMatrix  = Eigen::Matrix<double, NUM_JOINT, NUM_JOINT>; // NUM_JOINT x NUM_JOINT

private:
    pinocchio::Model _model;
    pinocchio::Data  _data;

    pinocchio::FrameIndex frame_id;
    int n; // total dof (NUM_LEG * NUM_JOINT)

public:
    PinocchioInterface(const std::string urdf_file,
                       const std::vector<std::string> foot_name)
        : urdf_file_(urdf_file), foot_name_(foot_name)
    {
        pinocchio::urdf::buildModel(urdf_file_, _model);
        pinocchio::Data data(_model);
        _data = data;

        n = NUM_LEG * NUM_JOINT;
        Initialize();
    };

    virtual ~PinocchioInterface() {}

public:
    void Initialize();

    void SetRobotBodyParameter(Eigen::Vector3d body_pos,
                               Eigen::Vector3d body_vel,
                               Eigen::Matrix3d body_rot);

    void SetRobotParameter(Eigen::VectorXd q, Eigen::VectorXd dq);

    void SetKinematics(int legtype);
    void SetJacobian(int legtype);

    void SetGravity();

    // (x,y,z) 기준 task space PD
    // pos_d/vel_d/acc_d : [3 x NUM_LEG] (각 열이 각 다리 목표값)
    void SetDynamics(const Eigen::Matrix<double, kTaskDim, NUM_LEG>& pos_d,
                     const Eigen::Matrix<double, kTaskDim, NUM_LEG>& vel_d,
                     const Eigen::Matrix<double, kTaskDim, NUM_LEG>& acc_d,
                     const TaskMatrix& Kp,
                     const TaskMatrix& Kd);

    // 기존 코드 호환(래퍼): pos_d/vel_d/acc_d가 MatrixXd, Kp/Kd가 Matrix4d로 들어오던 경우
    void SetDynamics(Eigen::MatrixXd pos_d,
                     Eigen::MatrixXd vel_d,
                     Eigen::MatrixXd acc_d,
                     Eigen::Matrix4d Kp,
                     Eigen::Matrix4d Kd);

    Eigen::Vector3d Getrpy(int legtype) const { return rpy[legtype]; }

    TaskVector GetPos(int legtype) const { return pos_[legtype]; }
    TaskVector GetVel(int legtype) const { return vel_[legtype]; }
    TaskVector GetAcc(int legtype) const { return acc_[legtype]; }

    TaskJacobian GetJacobian(int legtype) const { return J_[legtype]; }
    TaskJacobian GetJacobianDot(int legtype) const { return dJ_[legtype]; }

    Eigen::VectorXd GetDynamics();

    Eigen::VectorXd GetNLEffects() const { return r_B_; }
    Eigen::VectorXd GetGravityCompensation() const { return r_G_; }
    Eigen::VectorXd GetTau() const { return r_T_; }

    // 3D task-space 버전
    void SetTaskspacePD(const TaskMatrix& Kp,
                        const TaskMatrix& Kd,
                        const TaskVector pos_d[NUM_LEG],
                        const TaskVector vel_d[NUM_LEG],
                        const TaskVector acc_d[NUM_LEG]);

    // 기존 코드 호환(래퍼)
    void SetTaskspacePD(Eigen::MatrixXd Kp,
                        Eigen::MatrixXd Kd,
                        Eigen::VectorXd pos_d[],
                        Eigen::VectorXd vel_d[],
                        Eigen::VectorXd acc_d[]);

    void ComputeCTM();

    JointVector GetTorque(int i) const { return target_torque_leg_[i]; }

private:
    // Variables for Pinocchio
    std::string urdf_file_;
    std::vector<std::string> foot_name_;

    Eigen::VectorXd _q, _dq, _ddq;

    // (x,y,z)
    TaskVector   _pos;
    TaskJacobian _J, _dJ;

    // Actual Joint States
    JointVector q_[NUM_LEG], dq_[NUM_LEG], ddq_[NUM_LEG];

    // Actual Foot Position/Velocity/Acceleration (x,y,z)
    TaskVector pos_[NUM_LEG], vel_[NUM_LEG], acc_[NUM_LEG];

    // Jacobian (3 x NUM_JOINT)
    TaskJacobian J_[NUM_LEG], dJ_[NUM_LEG];

    // Dynamics terms (joint space)
    JointMatrix M_[NUM_LEG], C_[NUM_LEG];
    JointVector G_[NUM_LEG], B_[NUM_LEG], T_[NUM_LEG];

    // Desired task space (x,y,z)
    TaskVector pos_d_[NUM_LEG], vel_d_[NUM_LEG], acc_d_[NUM_LEG];

    // Task error (x,y,z)
    TaskVector err_[NUM_LEG], err_dot_[NUM_LEG];

    // Gains (3x3)
    TaskMatrix Kp_, Kd_;

    // Target torque
    Eigen::VectorXd target_torque_all_; // (NUM_LEG*NUM_JOINT)
    JointVector     target_torque_leg_[NUM_LEG];

    // return buffers
    TaskVector   r_pos_, r_vel_, r_acc_;
    TaskJacobian r_J_, r_dJ_;
    Eigen::MatrixXd r_M_, r_C_;
    Eigen::VectorXd r_B_, r_G_, r_T_;

    // WBC / Base state
    bool isWBC;
    std::string base_state_;

    Eigen::Vector3d body_pos_, body_vel_;
    Eigen::Vector3d body_rpy_, body_rpy_dot_;
    Eigen::Matrix3d rot_matrix;

    Eigen::Vector3d rpy[NUM_LEG];
};

#endif
