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
    NUM_MODE = 2,
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
    void StateLegCallback(const sensor_msgs::JointState &state);
    void SendCommandsToRobot();
    void Forward_Kinematics(Eigen::VectorXd, Eigen::VectorXd);

    ros::NodeHandle nh_;
    ros::Subscriber sub_leg_state_;

    ros::Publisher pub_leg_cmd_;

    const std::string go2_topic_leg_state_, go2_topic_leg_command_;

    ControlMode controlmode;

    Eigen::VectorXd q_               { Eigen::VectorXd::Zero(12) };
    Eigen::VectorXd dq_              { Eigen::VectorXd::Zero(12) };
    Eigen::VectorXd torque_          { Eigen::VectorXd::Zero(12) };

    Eigen::VectorXd q_current        { Eigen::VectorXd::Zero(12) };
    Eigen::VectorXd q_final          { Eigen::VectorXd::Zero(12) };
    Eigen::VectorXd q_desired        { Eigen::VectorXd::Zero(12) };


    Eigen::Vector3d EE_Pose_FL       { Eigen::Vector3d::Zero() };
    Eigen::Vector3d EE_Pose_FR       { Eigen::Vector3d::Zero() };
    Eigen::Vector3d EE_Pose_RL       { Eigen::Vector3d::Zero() };
    Eigen::Vector3d EE_Pose_RR       { Eigen::Vector3d::Zero() };

    bool Recieved_Joint_State;

    ros::Time Homing_Time;
    // int Homing_Time = 0;

};

#endif
