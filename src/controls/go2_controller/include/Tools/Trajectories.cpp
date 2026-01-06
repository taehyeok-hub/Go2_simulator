#include "Trajectories.hpp"

Trajectories::Trajectories()
{
    
}

Trajectories::~Trajectories() {}

Eigen::VectorXd Trajectories::Sinusoidal_Joint(int tick, double period, Eigen::VectorXd q_start_, Eigen::VectorXd q_final_)
{
    /* tick : 계속 변하는 변수 , period : 한 주기 */
    double phase = (double)tick / period;
    q_desired = q_start_ + (q_final_ - q_start_) * 0.5 * (1 - cos(M_PI * phase));

    return q_desired;
}

Eigen::Vector3d Trajectories::Sinusoidal_Task(int tick, double period, Eigen::Vector3d EE_start_, Eigen::Vector3d EE_final_)
{
    /* tick : 계속 변하는 변수 , period : 한 주기 */
    double phase = (double)tick / period;
    EE_desired = EE_start_ + (EE_final_ - EE_start_) * 0.5 * (1 - cos(M_PI * phase));

    return EE_desired;
}

Eigen::Vector3d Trajectories::Sinusoidal_Task_Rotate(int tick, double period, Eigen::VectorXd EE_start_, Eigen::VectorXd EE_final_)
{
    /* tick : 계속 변하는 변수 , period : 한 주기 */
    double phase = (double)tick / period;
    EE_desired = EE_start_ + (EE_final_ - EE_start_) * 0.5 * (1 - cos(2.0 * M_PI * phase));

    return EE_desired;
}

void Trajectories::Quintic_Joint(int tick, double motion_time, Eigen::VectorXd q_start_, Eigen::VectorXd q_final_)
{
    double t = (double)tick / (double)FREQUENCY;
    double T = motion_time; // 궤적 시간

    double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;

    // quintic trajectory : q = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    // M 행렬 에서 t에 관여
    // 초기 시간(t=0) : 위치, 속도, 가속도 (+) 도착 시간(t=T) : 위치, 속도, 가속도
    Eigen::Matrix<double, 6, 6> M;
    Eigen::Matrix<double, 6, 12> B;

    M << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 2, 0, 0, 0,
         1, T, T2, T3, T4, T5,
         0, 1, 2*T, 3*T2, 4*T3, 5*T4,
         0, 0, 2, 6*T, 12*T2, 20*T3;

    B.row(0) = q_start_.transpose();
    B.row(1) = Eigen::RowVectorXd::Zero(12);
    B.row(2) = Eigen::RowVectorXd::Zero(12);
    B.row(3) = q_final_.transpose();
    B.row(4) = Eigen::RowVectorXd::Zero(12);
    B.row(5) = Eigen::RowVectorXd::Zero(12);

    // MA = B 공식임.
    // A 행렬 : 공식의 계수들이 모여있는 행렬.
    Eigen::Matrix<double, 6, 12> A = M.inverse() * B;

    if (t > T)
    {
        // t >= T일 때 q_desired = q_final_ 이므로
        q_desired = q_final_;
        dq_desired.setZero();
    }
    else if (t < 0)
    {
        // t < 0 일 때 q_desired = q_start_ 이므로
        q_desired = q_start_;
        dq_desired.setZero();
    }

    double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;

    // 위치 계산: q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    Eigen::Matrix<double, 1, 6> t_vec_pos;
    t_vec_pos << 1, t, t2, t3, t4, t5;
    q_desired = (t_vec_pos * A).transpose();

    // 속도 계산: v(t) = a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4
    Eigen::Matrix<double, 1, 6> t_vec_vel;
    t_vec_vel << 0, 1, 2 * t, 3 * t2, 4 * t3, 5 * t4;
    dq_desired = (t_vec_vel * A).transpose();
}

QuinticTask Trajectories::Quintic_Task(int tick, double motion_time, Eigen::Vector3d EE_start_, Eigen::Vector3d EE_final_)
{
    double t = (double)tick / (double)FREQUENCY;
    double T = motion_time; // 궤적 시간

    double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;

    // quintic trajectory : q = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    // M 행렬 에서 t에 관여
    // 초기 시간(t=0) : 위치, 속도, 가속도 (+) 도착 시간(t=T) : 위치, 속도, 가속도
    Eigen::Matrix<double, 6, 6> M;
    Eigen::Matrix<double, 6, 3> B;

    M << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 2, 0, 0, 0,
         1, T, T2, T3, T4, T5,
         0, 1, 2*T, 3*T2, 4*T3, 5*T4,
         0, 0, 2, 6*T, 12*T2, 20*T3;

    B.row(0) = EE_start_.transpose();
    B.row(1) = Eigen::RowVector3d::Zero();
    B.row(2) = Eigen::RowVector3d::Zero();
    B.row(3) = EE_final_.transpose();
    B.row(4) = Eigen::RowVector3d::Zero();
    B.row(5) = Eigen::RowVector3d::Zero();

    // MA = B 공식임.
    // A 행렬 : 공식의 계수들이 모여있는 행렬.
    Eigen::Matrix<double, 6, 3> A = M.inverse() * B;

    if (t > T)
    {
        // t >= T일 때 x_desired = EE_final_ 이므로
        Desired.position = EE_final_;
        Desired.velocity.setZero();
        return Desired;
    }
    else if (t < 0)
    {
        // t < 0 일 때 x_desired = EE_start_ 이므로
        Desired.position = EE_start_;
        Desired.velocity.setZero();
        return Desired;
    }

    double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;

    // 위치 계산: q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    Eigen::Matrix<double, 1, 6> t_vec_pos;
    t_vec_pos << 1, t, t2, t3, t4, t5;
    Desired.position = (t_vec_pos * A).transpose();

    // 속도 계산: v(t) = a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4
    Eigen::Matrix<double, 1, 6> t_vec_vel;
    t_vec_vel << 0, 1, 2 * t, 3 * t2, 4 * t3, 5 * t4;
    Desired.velocity = (t_vec_vel * A).transpose();

    return Desired;
}



QuinticTask Trajectories::Quintic_Task_rostime(ros::Time &start_time, double motion_time, const Eigen::Vector3d &x_current, const Eigen::Vector3d &x_final)
{
    /*
    parameter 정리
    1. start_time : trajectory planning 시작 시간
    2. motion_time : trajectory planning 구동 시간
    3. x_current : 현재 발 끝 위치 (EE_Pose_FL,FR,RL,RR) (trajectory의 시작이 될 지점(x_start))
    4. x_final : 목표 발 끝 위치
    */

    Eigen::Vector3d x_ref = Eigen::Vector3d::Zero();

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
        0, 1, 2 * T, 3 * T2, 4 * T3, 5 * T4,
        0, 0, 2, 6 * T, 12 * T2, 20 * T3;

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
        Desired.position = x_final;
        Desired.velocity.setZero();
        return Desired;
    }
    else if (t < 0)
    {
        // t < 0 일 때 x_desired = x_current 이므로
        Desired.position = x_current;
        Desired.velocity.setZero();
        return Desired;
    }

    double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;

    // 위치 계산: q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    Eigen::Matrix<double, 1, 6> t_vec_pos;
    t_vec_pos << 1, t, t2, t3, t4, t5;
    Desired.position = (t_vec_pos * A).transpose();

    // 속도 계산: v(t) = a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4
    Eigen::Matrix<double, 1, 6> t_vec_vel;
    t_vec_vel << 0, 1, 2 * t, 3 * t2, 4 * t3, 5 * t4;
    Desired.velocity = (t_vec_vel * A).transpose();

    //     // 0 < t < T 일 때
    //     double t2 = t*t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
    //     Eigen::Matrix<double, 1, 6> t_vec;
    //     t_vec << 1, t, t2, t3, t4, t5;

    //     // t_vec은 1x6 / A는 6x3 -> 따라서 x_desired는 1x3이 되고 이를 transpose 하니까 3x1이 된다.
    //     x_desired = (t_vec * A).transpose();
    //     return x_desired;
    // }
    return Desired;
}

