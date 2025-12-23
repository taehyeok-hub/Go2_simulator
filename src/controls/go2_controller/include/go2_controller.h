#ifndef GO2_CONTROLLER_H_
#define GO2_CONTROLLER_H_

#include "Pinocchio_Interface.hpp"

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <std_srvs/Empty.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ignition/math/Vector3.hh>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <cmath>
#include <thread>
#include <vector>
#include <array>
#include <chrono>
#include <iostream>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "SingleRigidBody.hpp"
#include "Kinematics.hpp"
#include "Trajectories.hpp"
#include "Centroidal_Dynamics.hpp"
// #include "Centt.hpp"
#include "Enum_Shared.hpp"

enum ControlMode
{
    INIT,
    HOMING,
    SQUATING,
    POSTURE,
    NUM_MODE = 5,
};

std::string urdf_path_ = "/home/pth/pth/model/Go2_simulator/src/robots/go2_descriptions/urdf/go2.urdf";
std::vector<std::string> foot_name_ = {"FL_foot", "FR_foot", "RL_foot", "RR_foot"};

PinocchioInterface PINO(urdf_path_, foot_name_);

class go2_controller
{
public:
    go2_controller(ros::NodeHandle &nh,
                   std::string go2_topic_leg_state,
                   std::string go2_topic_leg_command,
                   const double freq)
        : nh_(nh),
          go2_topic_leg_state_(go2_topic_leg_state),
          go2_topic_leg_command_(go2_topic_leg_command)
    {
    }
    virtual ~go2_controller() {}

    void Init();
    void Run();
    void CentRun();
    void PlotRun();

private:
    // FUNCTION---------------------------------------------------------------------------------------------------------------------------------------------------

    void Command(bool flag);
    void StateBodyCallback(const gazebo_msgs::ModelStates::ConstPtr &body);
    void StateLegCallback(const sensor_msgs::JointState &state);
    void SendCommandsToRobot();
    void DataStream();

    void Homing();
    void Squating();
    void SRBMControl();
    void Reference_Generator();
    void Posture_Control();
    void Forward_Kinematics(const Eigen::VectorXd &q, const Eigen::VectorXd &dq);
    void Forward_Kinematics_ME(const Eigen::VectorXd &q, const Eigen::VectorXd &dq);
    void TaskSpacePDControl(double Kp, double Kd);
    void Set_Kinematics();


    // Trajectory 관련 함수들
    TrajectoryPoint Quintic_Task(ros::Time &start_time, double motion_time, Eigen::Vector3d &x_current, Eigen::Vector3d &x_final);
    Eigen::VectorXd Quintic_Joint(ros::Time &start_time, double motion_time, Eigen::VectorXd &q, Eigen::VectorXd &qf);
    // Eigen::Vector3d Sinusoidal_Task();
    // Eigen::VectorXd Sinusoidal_Joint();

    // CLASS---------------------------------------------------------------------------------------------------------------------------------------------------

    Kinematics KINE;
    Trajectories PLAN;
    SingleRigidBody SRBM;
    Centroidal_Dynamics CENT;
    // Centt CENT;
    

    // VARIABLE---------------------------------------------------------------------------------------------------------------------------------------------------

    // ros에서 노드 핸들러와 구독자, 발행자
    ros::NodeHandle nh_;
    ros::Subscriber sub_body_states_, sub_leg_state_;
    ros::Publisher pub_leg_cmd_;
    ros::Publisher pub_TH_;

    const std::string go2_topic_leg_state_, go2_topic_leg_command_;

    ControlMode controlmode;

    // (시뮬레이션에서 받아오는) 로봇의 상태 변수
    Eigen::VectorXd gazebo_body_pos, gazebo_body_vel;
    Eigen::VectorXd gazebo_quat;
    Eigen::VectorXd gazebo_rpy, gazebo_rpy_dot;

    Eigen::VectorXd q_;
    Eigen::VectorXd dq_;
    Eigen::VectorXd torque_;

    // Joint space 변수들 (quintic)
    int Start_Flag = 0;
    int Init_Time = 0;
    Eigen::VectorXd q_start;    
    Eigen::VectorXd q_final;
    Eigen::VectorXd q_desired; // Planning 에서 Trajectory 받아오는 곳.
    Eigen::VectorXd Start_Position, Homing_Position;
    // Eigen::VectorXd q_desired{Eigen::VectorXd::Zero(12)}; // trajectory가 들어갈 곳

    // Eigen::Vector3d Body_Pos;
    // Eigen::Matrix3d Body_Rot;
    // Eigen::Vector3d Body_Ref;

    Eigen::VectorXd COM_Ref; // Reference 값 넣어두는 용도
    
    // Reference Generator
    double Pos_Command[NUM_AXIS] = {0.0, 0.0, 0.0};
    double Vel_Command[NUM_AXIS] = {0.0, 0.0, 0.0};
    double RPY_Command[NUM_AXIS] = {0.0, 0.0, 0.0};
    double ANG_Command[NUM_AXIS] = {0.0, 0.0, 0.0};

    Eigen::MatrixXd Foot_J[NUM_LEG];
    Eigen::VectorXd Foot_Pos[NUM_LEG], Foot_Vel[NUM_LEG];
    Eigen::VectorXd Torque[NUM_LEG];


    // (in Task Space) 동작의 시작 xyz값 (담아두는 용도)
    Eigen::Vector3d EE_Pose_FL_start, EE_Pose_FR_start, EE_Pose_RL_start, EE_Pose_RR_start;

    // (in Task Space) 동작의 최종 목표의 xyz값
    Eigen::Vector3d EE_Pose_FL_final, EE_Pose_FR_final, EE_Pose_RL_final, EE_Pose_RR_final;

    // Task Space - 궤적과 PD제어기 사이의 "통신 채널"
    Eigen::Vector3d EE_Pose_FL_desired, EE_Pose_FR_desired, EE_Pose_RL_desired, EE_Pose_RR_desired;
    Eigen::Vector3d EE_Vel_FL_desired, EE_Vel_FR_desired, EE_Vel_RL_desired, EE_Vel_RR_desired;

    Eigen::Matrix3d Kp_Task{Eigen::Matrix3d::Zero()};
    Eigen::Matrix3d Kd_Task{Eigen::Matrix3d::Zero()};

    

    // Jacobians
    Eigen::Matrix<double, 6, 3> J_FL{Eigen::Matrix<double, 6, 3>::Zero()};
    Eigen::Matrix<double, 6, 3> J_FR{Eigen::Matrix<double, 6, 3>::Zero()};
    Eigen::Matrix<double, 6, 3> J_RL{Eigen::Matrix<double, 6, 3>::Zero()};
    Eigen::Matrix<double, 6, 3> J_RR{Eigen::Matrix<double, 6, 3>::Zero()};
    Eigen::Matrix<double, 6, 12> J{Eigen::Matrix<double, 6, 12>::Zero()};
    Eigen::Matrix<double, 6, 3> Jacobian[NUM_LEG];

    Eigen::Matrix3d R_bw_;
    Eigen::VectorXd GRF;
    Eigen::Vector3d Force_[NUM_LEG];

    // 동작 상태 관리
    ros::Time motion_start_time_;
    bool is_motion_started_ = false;
    bool is_going_down_ = true;
    std::array<Eigen::Vector3d, 4> traj_start_poses_;
    std::array<Eigen::Vector3d, 4> traj_final_poses_;
    int squat_count = 0;

    
    bool Recieved_Joint_State;

    ros::Time Motion_Time;
    ros::Time Current_Time;
    // int Homing_Time = 0;

    TrajectoryPoint Sinusoidal_Task(ros::Time &start_time, double period,
                                    const Eigen::Vector3d &stand_pose,
                                    const Eigen::Vector3d &squat_pose);

    int count = 0;
};

#endif

// // using 키워드 (C++11 이상에서 선호)
// using Matrix6x12d = Eigen::Matrix<double, 6, 12>;
// Matrix6x12d J = Matrix6x12d::Zero();

// // 또는 auto와 함께
// auto J = Matrix6x12d::Zero();