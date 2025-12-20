#include <iostream>
#include <ros/ros.h>

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "Kinematics.hpp"

// 궤적의 결과를 담을 구조체
struct TrajectoryPoint
{
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    // Eigen::Vector3d acceleration;
};


class Trajectories
{
private:
    // Constant ------------------------------------------------------------------------------------------------------------------------------------------------------------------


    // Variables ------------------------------------------------------------------------------------------------------------------------------------------------------------------

    Eigen::VectorXd q_desired;
    Eigen::VectorXd dq_desired;
    
    Eigen::MatrixXd M_Matirx;
    Eigen::MatrixXd A_Matrix;
    Eigen::MatrixXd B_Vector;

    // Serve Function ------------------------------------------------------------------------------------------------------------------------------------------------------------------

    

public:
    // Constructor
    Trajectories();
    ~Trajectories();
    
    Eigen::MatrixXd Set_Coefficient_QuinticJoint()

    // Public Variables
    
    
    
    // Public Function ------------------------------------------------------------------------------------------------------------------------------------------------------------------
    TrajectoryPoint Quintic_Task(ros::Time &start_time, double motion_time, const Eigen::Vector3d &x_current, const Eigen::Vector3d &x_final);
    
    Eigen::VectorXd Quintic_Joint(double t, double motion_time, const Eigen::VectorXd &q_start, const Eigen::VectorXd &q_final);

    TrajectoryPoint Sinusoidal_Task(ros::Time &start_time, double period,
                                    const Eigen::Vector3d &stand_pose, const Eigen::Vector3d &squat_pose);
    
    // Eigen::VectorXd Sinusoidal_Joint();


    static inline double Set_TimeVariable(ros::Time Current_Time_, ros::Time Motion_Time)
    {
        double t = (Current_Time_ - Motion_Time_).toSec();
        return t;
    }; 

    Eigen::MatrixXd(double Motion_Time_, )
};