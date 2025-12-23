#include <iostream>
#include <ros/ros.h>

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "Kinematics.hpp"

enum Control
{
    JOINT,
    TASK,
    NUM_CONTROL=2,
};

// 궤적의 결과를 담을 구조체 (여러개의 값을 반환하고 싶을 때 사용함.)
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

    double frequency;

    // Variables ------------------------------------------------------------------------------------------------------------------------------------------------------------------
    
    double period;
    bool Is_Planned = false;
    Eigen::VectorXd q_desired;
    Eigen::VectorXd dq_desired;
    
    Eigen::MatrixXd M_Matrix;
    Eigen::MatrixXd A_Matrix;
    Eigen::MatrixXd B_Vector;

    // Serve Function ------------------------------------------------------------------------------------------------------------------------------------------------------------------

    TrajectoryPoint Desired;

public:
    // Constructor
    Trajectories();
    ~Trajectories();

    // Public Function ------------------------------------------------------------------------------------------------------------------------------------------------------------------
    TrajectoryPoint Quintic_Task(ros::Time &start_time, double motion_time, const Eigen::Vector3d &x_current, const Eigen::Vector3d &x_final);
    
    Eigen::VectorXd Quintic_Joint(double t, double motion_time, const Eigen::VectorXd &q_start, const Eigen::VectorXd &q_final);

    TrajectoryPoint Sinusoidal_Task(ros::Time &start_time, double period,
                                    const Eigen::Vector3d &stand_pose, const Eigen::Vector3d &squat_pose);
    


    // Get 함수
    Eigen::MatrixXd Get_Coefficient_Matrix() {return A_Matrix; };
};