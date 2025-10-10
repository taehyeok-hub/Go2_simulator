#ifndef GO2_CONTROLLER_H_
#define GO2_CONTROLLER_H_

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
#include <array>

#include <cmath>
#include <thread>
#include <chrono>
#include <iostream>

#include "Eigen/Dense"
#include "Eigen/Geometry"

enum Joint
{
    HR,
    HP,
    KP,
    NUM_JOINT = 3,
};

enum Leg_Num
{
    FL,
    FR,
    RL,
    RR,
    NUM_LEG = 4,
};

enum ControlMode
{
    INIT,
    HOMING,
    SQUATING,
    NUM_MODE = 3,
};

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


private:
    // FUNCTION---------------------------------------------------------------------------------------------------------------------------------------------------

    void Command(bool flag);
    void Homing();
    void Squating();
    void StateLegCallback(const sensor_msgs::JointState &state);
    void SendCommandsToRobot();
    void Forward_Kinematics(const Eigen::VectorXd& q, const Eigen::VectorXd& dq);
    void geometrical_IK();
    void Jacobians_URDF(const Eigen::VectorXd& q);
    void Create_Jacobian(const Eigen::VectorXd& q);
    void TaskSpacePDControl(double Kp, double Kd);


    // Trajectory 관련 함수들
    Eigen::Vector3d Quintic_Task(ros::Time& start_time, double motion_time, Eigen::Vector3d& x_current, Eigen::Vector3d& x_final);
    Eigen::VectorXd Quintic_Joint(ros::Time& start_time, double motion_time, Eigen::VectorXd& q, Eigen::VectorXd& qf);  
    Eigen::Vector3d Sinusoidal_Task();
    Eigen::VectorXd Sinusoidal_Joint();     


    ros::NodeHandle nh_;
    ros::Subscriber sub_leg_state_;

    ros::Publisher pub_leg_cmd_;

    const std::string go2_topic_leg_state_, go2_topic_leg_command_;

    ControlMode controlmode;
 
    // (시뮬레이션에서 받아오는) 로봇의 현재 상태 변수
    Eigen::VectorXd q_               { Eigen::VectorXd::Zero(12) };
    Eigen::VectorXd dq_              { Eigen::VectorXd::Zero(12) };
    Eigen::VectorXd torque_          { Eigen::VectorXd::Zero(12) };

    // Joint space 변수들 (quintic)
    Eigen::VectorXd q_current        { Eigen::VectorXd::Zero(12) };
    Eigen::VectorXd q_final          { Eigen::VectorXd::Zero(12) };
    Eigen::VectorXd q_desired        { Eigen::VectorXd::Zero(12) }; // trajectory가 들어갈 곳

    // FK 결과 (현재 발 끝 위치)
    Eigen::Vector3d EE_Pose_FL       { Eigen::Vector3d::Zero() };
    Eigen::Vector3d EE_Pose_FR       { Eigen::Vector3d::Zero() };
    Eigen::Vector3d EE_Pose_RL       { Eigen::Vector3d::Zero() };
    Eigen::Vector3d EE_Pose_RR       { Eigen::Vector3d::Zero() };

    // (in Task Space) 동작의 최종 목표의 xyz값
    Eigen::Vector3d EE_Pose_FL_final, EE_Pose_FR_final, EE_Pose_RL_final, EE_Pose_RR_final;

    // Task Space - 궤적과 PD제어기 사이의 "통신 채널"
    Eigen::Vector3d EE_Pose_FL_desired, EE_Pose_FR_desired, EE_Pose_RL_desired, EE_Pose_RR_desired;
    Eigen::Vector3d EE_Vel_FL_desired, EE_Vel_FR_desired, EE_Vel_RL_desired, EE_Vel_RR_desired;

    Eigen::Matrix3d Kp_Task         { Eigen::Matrix3d::Zero() };
    Eigen::Matrix3d Kd_Task         { Eigen::Matrix3d::Zero() };
    
    Eigen::VectorXd all_joint_angles { Eigen::VectorXd::Zero(12) }; 

    // Jacobians
    Eigen::Matrix<double,6,3> J_FL   {Eigen::Matrix<double,6,3>::Zero()};
    Eigen::Matrix<double,6,3> J_FR   {Eigen::Matrix<double,6,3>::Zero()};
    Eigen::Matrix<double,6,3> J_RL   {Eigen::Matrix<double,6,3>::Zero()};
    Eigen::Matrix<double,6,3> J_RR   {Eigen::Matrix<double,6,3>::Zero()};
    Eigen::Matrix<double,6,12> J     {Eigen::Matrix<double,6,12>::Zero()};

    // 동작 상태 관리
    ros::Time motion_start_time_;
    bool is_motion_started_ = false;
    std::array<Eigen::Vector3d, 4> traj_start_poses_;
    std::array<Eigen::Vector3d, 4> traj_final_poses_;
    
    bool Recieved_Joint_State;

    ros::Time Homing_Time;
    // int Homing_Time = 0;

};

#endif


// // using 키워드 (C++11 이상에서 선호)
// using Matrix6x12d = Eigen::Matrix<double, 6, 12>;
// Matrix6x12d J = Matrix6x12d::Zero();

// // 또는 auto와 함께
// auto J = Matrix6x12d::Zero();