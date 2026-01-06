#ifndef GO_CONTROLLER_H_
#define GO_CONTROLLER_H_

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
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ContactsState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <cmath>
#include <thread>
#include <mutex>
#include <chrono>
#include <fstream>
#include <algorithm>
#include <memory>
#include <sstream>
#include <string>
#include <iostream>
#include <vector>

#include "Eigen/Dense"


class plate_bot_controller
{
public:
    plate_bot_controller(ros::NodeHandle &nh,
                   std::string plate_bot_topic_leg_state,
                   std::string plate_bot_topic_leg_command,
                   const double freq)
        : nh_(nh),
          plate_bot_topic_leg_state_(plate_bot_topic_leg_state),
          plate_bot_topic_leg_command_(plate_bot_topic_leg_command),
          freq_(freq),
          pi(M_PI),
          deg2rad(pi / 180),
          rad2deg(180 / pi)
    {
    }
    virtual ~plate_bot_controller() {}

    void Init();
    void Run();

private:
    // FUNCTION---------------------------------------------------------------------------------------------------------------------------------------------------

    void Command(bool flag);
    void StateLegCallback(const sensor_msgs::JointState &state);
    void SendCommandsToRobot();
    void DataStream();
    double Rotating_Command(double period, Eigen::VectorXd upper_radian, Eigen::VectorXd lower_radian); 

    // CLASS---------------------------------------------------------------------------------------------------------------------------------------------------

    // VARIABLE---------------------------------------------------------------------------------------------------------------------------------------------------

    ros::NodeHandle nh_;

    ros::Subscriber sub_leg_state_;

    ros::Publisher pub_leg_cmd_, pub_real_data_, pub_ref_data_, pub_data_;

    const double pi, deg2rad, rad2deg, freq_;
    const std::string plate_bot_topic_leg_state_, plate_bot_topic_leg_command_;

    bool Recieved_Joint_State;
    double current_time_, Roll_time, Pitch_time;
    int Roll_flag = 0, Pitch_flag = 0;
    double period_roll, period_pitch;

    Eigen::VectorXd q_, dq_, torque_, target_joint_pos_, currentPosition, Plate_angle, Upper_Angle, Lower_Angle;

};

#endif