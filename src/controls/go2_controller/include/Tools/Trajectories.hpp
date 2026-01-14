#include <iostream>
#include <ros/ros.h>

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "Kinematics.hpp"

enum Control
{
    JOINT,
    TASK,
    NUM_CONTROL = 2,
};

// 궤적의 결과를 담을 구조체 (여러개의 값을 반환하고 싶을 때 사용함.)
struct QuinticTask
{
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
};

struct QuinticJoint
{
    Eigen::VectorXd position;
    Eigen::VectorXd velocity;
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

    Eigen::Vector3d EE_desired;

    Eigen::MatrixXd M_Matrix_Joint;
    Eigen::MatrixXd A_Matrix_Joint;
    Eigen::MatrixXd B_Vector_Joint;

    Eigen::MatrixXd M_Matrix_Task;
    Eigen::MatrixXd A_Matrix_Task;
    Eigen::MatrixXd B_Vector_Task;

    double x_target, y_target;

    // Serve Function ------------------------------------------------------------------------------------------------------------------------------------------------------------------

    QuinticTask Desired;
    // QuinticJoint Desired;

public:
    // Constructor
    Trajectories();
    ~Trajectories();

    // Public Function ------------------------------------------------------------------------------------------------------------------------------------------------------------------
    void SetQuinticJoint();
    void SetQuinticTask();
    double Raibert_Heuristic();
    double Sinusoidal(int tick, double start_, double final_);
    double Sinusoidal_D(int tick, double start_, double final_);
    Eigen::VectorXd Sinusoidal_Joint(int tick, double period, Eigen::VectorXd q_start_, Eigen::VectorXd q_final_);
    Eigen::VectorXd Sinusoidal_Joint_D(int tick, double period, Eigen::VectorXd q_start_, Eigen::VectorXd q_final_);
    Eigen::Vector3d Sinusoidal_Task(int tick, double period, Eigen::Vector3d EE_start_, Eigen::Vector3d EE_final_);
    Eigen::Vector3d Sinusoidal_Task_D(int tick, double period, Eigen::Vector3d EE_start_, Eigen::Vector3d EE_final_);
    Eigen::Vector3d Sinusoidal_Task_Rotate(int tick, double period, Eigen::VectorXd EE_start_, Eigen::VectorXd EE_final_);
    QuinticTask Quintic_Task(int tick, double motion_time, Eigen::Vector3d EE_start_, Eigen::Vector3d EE_final_);
    void Quintic_Joint(int tick, double motion_time, Eigen::VectorXd q_start_, Eigen::VectorXd q_final_);
    QuinticTask Quintic_Task_rostime(ros::Time &start_time, double motion_time, const Eigen::Vector3d &x_current, const Eigen::Vector3d &x_final);
    double Quintic(double t, double T ,double q0, double q1);
    double QuinticD(double t, double T, double q0, double q1);
    double RaibertX(int leg, double p_com, double v_com, double T_Gait);
    double RaibertY(int leg, double p_com, double v_com, double T_Gait);
    Eigen::Vector3d Raibert_Heuristic(int leg, Eigen::Vector3d p_com, Eigen::Vector3d v_com, double T_Gait);

    Eigen::MatrixXd Set_M_Matrix(double T)
    {
        double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
        M_Matrix_Task << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 1, T, T2, T3, T4, T5, 0, 1, 2 * T, 3 * T2, 4 * T3, 5 * T4, 0, 0, 2, 6 * T, 12 * T2, 20 * T3;
        return M_Matrix_Task;
    };

    // Get 함수
    Eigen::VectorXd Get_Reference_Pose_QuinticJoint() { return q_desired; };
    Eigen::VectorXd Get_Reference_Vel_QuinticJoint() { return dq_desired; };
};