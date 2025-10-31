#include "go2_controller.h"

Eigen::Vector3d go2_controller::Quintic_Task(ros::Time& start_time, double motion_time, Eigen::Vector3d& x_current, 
    Eigen::Vector3d& x_final)
{
    /*
    parameter 정리
    1. start_time : trajectory planning 시작 시간
    2. motion_time : trajectory planning 구동 시간
    3. x_current : 현재 발 끝 위치 (EE_Pose_FL,FR,RL,RR) (trajectory의 시작이 될 지점(x_start))
    4. x_final : 목표 발 끝 위치
    */

    Eigen::Vector3d x_desired = Eigen::Vector3d::Zero();

    ros::Time Current_time = ros::Time::now();
    double t = (Current_time - start_time).toSec();
    double T = motion_time; // 궤적 시간
    
    double T2 = T*T, T3 = T2*T, T4 = T3*T, T5 = T4*T;
    
    // quintic trajectory : q = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    // M 행렬 에서 t에 관여
    // 초기 시간(t=0) : 위치, 속도, 가속도 (+) 도착 시간(t=T) : 위치, 속도, 가속도 

    Eigen::Matrix<double, 6, 6> M; 
    M << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 2, 0, 0, 0,
         1, T, T2, T3, T4, T5,
         0, 1, 2*T, 3*T2, 4*T3, 5*T4,
         0, 0, 2, 6*T, 12*T2, 20*T3;
    
    Eigen::Matrix<double, 6, 6> M_inv = M.inverse();


    // B 행렬 : [q0, v0, a0, qf, vf af].transpose() 느낌. 
    // 끝 관절의 x,y,z 축 좌표에서의 초기위치,속도,가속도 / 끝위치,속도,가속도
    Eigen::Matrix<double, 6, 3> B;
    B.row(0) = x_current.transpose();
    B.row(1) = Eigen::RowVector3d::Zero();
    B.row(2) = Eigen::RowVector3d::Zero();
    B.row(3) = x_final.transpose();
    B.row(4) = Eigen::RowVector3d::Zero();
    B.row(5) = Eigen::RowVector3d::Zero();

    // MA = B 공식임.
    // A 행렬 : 공식의 계수들이 모여있는 행렬.
    Eigen::Matrix<double, 6, 3> A = M_inv * B;

    if (t > T)
    {
        // t >= T일 때 x_desired = x_final 이므로
        return x_final; 
    }
    if (t < 0.01) // 동작 시작 직후 0.01초 동안만 출력
    {
        Eigen::Vector3d initial_error = EE_Pose_FL_desired - EE_Pose_FL;
        ROS_INFO_STREAM("Initial Squat Error FL: " << initial_error.transpose());
    }   
    else if (t < 0)
    {
        // t < 0 일 때 x_desired = x_current 이므로
        return x_current;
    }
    else
    {
        // 0 < t < T 일 때
        double t2 = t*t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
        Eigen::Matrix<double, 1, 6> t_vec;
        t_vec << 1, t, t2, t3, t4, t5;

        // t_vec은 1x6 / A는 6x3 -> 따라서 x_desired는 1x3이 되고 이를 transpose 하니까 3x1이 된다.
        x_desired = (t_vec * A).transpose();
        return x_desired;
    }
    
}


TrajectoryPoint go2_controller::Sinusoidal_Task(ros::Time& start_time, double period, 
                                                const Eigen::Vector3d& stand_pose, 
                                                const Eigen::Vector3d& squat_pose)
{
    double t = (ros::Time::now() - start_time).toSec();
    double T = period;

    // 1. Z축에 대한 파라미터 계산
    double z_stand = stand_pose.z();
    double z_squat = squat_pose.z();
    double z_amplitude = (z_stand - z_squat) / 2.0;
    double z_center = (z_stand + z_squat) / 2.0;

    // 2. 시간에 따른 목표(not 최종 목표) (desired) 위치, 속도 식
    double z_desired = z_center + z_amplitude * cos(2 * M_PI * t / T);
    double z_vel_desired = - z_amplitude * (2 * M_PI / T) * sin(2 * M_PI * t / T);

    // 3. X, Y는 고정되어있음.
    TrajectoryPoint target;
    target.position << stand_pose.x(), stand_pose.y(), z_desired;
    target.velocity << 0, 0, z_vel_desired;
    
    return target;
}


Eigen::VectorXd go2_controller::Quintic_Joint(ros::Time& start_time, double motion_time, Eigen::VectorXd& q, 
    Eigen::VectorXd& qf)
{
    q_current = q;
    q_final = qf;

    ros::Time Current_time = ros::Time::now();
    double t = (Current_time - start_time).toSec();
    double T = motion_time; // 궤적 시간
    
    double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
    
    // quintic trajectory : q = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    // M 행렬 에서 t에 관여
    // 초기 시간(t=0) : 위치, 속도, 가속도 (+) 도착 시간(t=T) : 위치, 속도, 가속도 

    Eigen::Matrix<double, 6, 6> M; 
    M << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 2, 0, 0, 0,
         1, T, T2, T3, T4, T5,
         0, 1, 2*T, 3*T2, 4*T3, 5*T4,
         0, 0, 2, 6*T, 12*T2, 20*T3;

    Eigen::Matrix<double, 6, 6> M_inv = M.inverse(); // M의 역행렬을 구함

    Eigen::Matrix<double, 6, 12> B; // 12개의 관절들의 초기 (위치, 속도, 가속도), 최종 (위치, 속도, 가속도) 구하기 
    B.row(0) = q_current.transpose(); // current를 받으면서 바로 초기화
    B.row(1) = Eigen::RowVectorXd::Zero(12);
    B.row(2) = Eigen::RowVectorXd::Zero(12);
    B.row(3) = q_final.transpose();
    B.row(4) = Eigen::RowVectorXd::Zero(12);
    B.row(5) = Eigen::RowVectorXd::Zero(12);

    Eigen::Matrix<double, 6, 12> A = M_inv * B; // 계수행렬을 구합니다.

    if (t >= T)
    {
        q_desired = q_final;
        return q_desired;
    }
    else 
    {
        // q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5 이므로
        double t2 = t * t; 
        double t3 = t2 * t; 
        double t4 = t3 * t; 
        double t5 = t4 * t;
        Eigen::Matrix<double, 1, 6> T_vec;
        T_vec << 1, t, t2, t3, t4, t5;

        q_desired = (T_vec * A).transpose(); // 12 x 1 열벡터가 된다.
        return q_desired;
    }
}

// Eigen::Vector3d go2_controller::Sinusoidal_Task()
// {
//     return;
// }

// Eigen::VectorXd go2_controller::Sinusoidal_Joint()
// {
//     return;
// }



// quintic trajectory in joint space
// ros::Time Current_Time = ros::Time::now();
// double t = (Current_Time - Homing_Time).toSec(); // toSec() 적으세요
// double T = 2.0;
// double time_ratio = t/T;
// double time_ratio_3 = time_ratio * time_ratio * time_ratio;
// double time_ratio_4 = time_ratio_3 * time_ratio;
// double time_ratio_5 = time_ratio_4 * time_ratio;

// if (t >= T)
// {
//     q_desired = q_final;
// }
// else if (t< T)
// {
//     q_desired = q_current + (q_final - q_current) * (10 * time_ratio_3 - 15 * time_ratio_4 + 6 * time_ratio_5);
// } 



// sinusoidal trajectory
// ros::Time Current_Time = ros::Time::now();
// double t = (Current_Time - Homing_Time).toSec(); // 계산해야하므로 ros::Time이 아닌 double로 받음.
// double T = 2; // 계산해야하므로 ros::Time이 아닌 double로 받음.
// if (t< T) // trajectory 추정중
// {
    // q_desired = q_current + (q_final - q_current) * 0.5 * (1 - cos(3.14 / T * t));
// }
// else if (t == T)
// {
    // q_desired = q_final;
// }
