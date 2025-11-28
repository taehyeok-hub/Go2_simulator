#include "go2_controller.h"

void go2_controller::Init()
{
    // Reset Gazebo Simualtion
    system("clear");
    std_srvs::Empty reset;
    ros::service::call("/gazebo/reset_simulation", reset);
    std::cout << "Reset Gazebo Simulation" << std::endl;
    nh_.ok();

    uint32_t queue_size = 10;

    // Subscribers
    sub_body_states_ = nh_.subscribe("/gazebo/model_states", queue_size, &go2_controller::StateBodyCallback, this, ros::TransportHints().reliable().tcpNoDelay());
    sub_leg_state_ = nh_.subscribe(go2_topic_leg_state_, queue_size, &go2_controller::StateLegCallback, this, ros::TransportHints().reliable().tcpNoDelay());
    // leg_state(다리상태)를 go2의 legstate를 구독함.
    /* StateLegCallback 함수는 코드상에서 직접 호출되는 부분이 없으며, ROS 시스템에 의해 특정 이벤트가 발생했을 때 자동으로 호출되도록 등록되어 있습니다. */

    // Publishers
    // pub_TH : plotjuggler로 Float의 형태로 본인의 상태를 발행하는거임.
    pub_TH_ = nh_.advertise<std_msgs::Float64MultiArray>("TH", queue_size);
    pub_leg_cmd_ = nh_.advertise<std_msgs::Float64MultiArray>(go2_topic_leg_command_, queue_size);

    controlmode = INIT;
    Recieved_Joint_State = false;

    q_.setZero(12);
    dq_.setZero(12);
    torque_.setZero(12);

    gazebo_body_pos.setZero(NUM_AXIS);
    gazebo_body_vel.setZero(NUM_AXIS);
    gazebo_quat.setZero(4);
    gazebo_rpy.setZero(NUM_AXIS);
    gazebo_rpy_dot.setZero(NUM_AXIS);

    // Joint Space PD 제어(Homing) : 0, 0.67, -1.40, 0, 0.67, -1.40, 0, 0.67, -1.40, 0, 0.67, -1.40
}

void go2_controller::Command(bool flag)
{
    if (Recieved_Joint_State)
    {
        KINE.Forward_Kinematics(q_, dq_);
        KINE.Jacobian(q_);
        std::cout << "몸통 좌표 : x = " << gazebo_body_pos(0) << " y = " << gazebo_body_pos(1) << " z = " << gazebo_body_pos(2) << std::endl;
        // Forward_Kinematics_ME(q_, dq_);
        // Create_Jacobian(q_);

        switch (controlmode)
        {
        case INIT:
        {
            // Homing 시작 시 초기 상태 저장
            q_current = q_;
            Moving_Time = ros::Time::now();
            controlmode = HOMING;
            break;
        }
        case HOMING:
        {
            Homing();
            if ((ros::Time::now() - Moving_Time).toSec() >= 4.0)
            {
                is_motion_started_ = false;
                // controlmode = SQUATING; ----> 이거 풀으면 다시 SQUATING 합니다.
                controlmode = SRBM_CONTROL;
            }
            break;
        }
        case SQUATING:
        {
            Squating();
            break;
        }
        case SRBM_CONTROL:
        {
            // double alpha = 0.02;
            // Body_Pos.setZero();
            // Body_Rot.setZero();
            // Body_Pos = (1 - alpha) * Body_Pos + alpha * gazebo_body_pos;
            // Body_Rot = SRBM.RPYRotationMatrix(gazebo_rpy(0), gazebo_rpy(1), gazebo_rpy(2));
            SRBMControl();
            break;
        }
        }

        SendCommandsToRobot();
    }
}

void go2_controller::Homing() // 초기자세 설정 하는 코드
{
    ros::Time Current_Time = ros::Time::now();

    double t = (Current_Time - Moving_Time).toSec(); // toSec() 적으세요
    double T;

    T = 1.0;
    q_final << 0, 1.20, -2.60, 0, 1.20, -2.60, 0, 1.20, -2.60, 0, 1.20, -2.60;
    Eigen::VectorXd q_desired = TRAJ.Quintic_Joint(t, T, q_current, q_final);
    double Kp = 100.0, Kd = 0.8;
    torque_ = Kp * (q_desired - q_) - Kd * dq_;

    T = 3.0;
    q_current = q_final;
    q_final << 0, 0.67, -1.40, 0, 0.67, -1.40, 0, 0.67, -1.40, 0, 0.67, -1.40;
    q_desired = TRAJ.Quintic_Joint(t, T, q_current, q_final);
    Kp = 100.0, Kd = 0.8;

    torque_ = Kp * (q_desired - q_) - Kd * dq_; // q_와 dq_는 계속 StateLegCallback 함수로 인해 실시간으로 값을 할당받는중임.
    // 0.1925, 0.145, -0.31
}

void go2_controller::Squating()
{
    // Sinusoidal_rotate
    // double motion_duration = 5.0; // 앉았다 일어서는 데 총 3초

    // if (is_motion_started_)
    // {
    //     double t = (ros::Time::now() - motion_start_time_).toSec();

    //     if (t > motion_duration)
    //     {
    //         is_motion_started_ = false;
    //     }
    // }
    // else
    // {
    //     motion_start_time_ = ros::Time::now();

    //     EE_Pose_FL_start << 0.20, 0.13, -0.30;   // 0.19, 0.1425, -0.30
    //     EE_Pose_FR_start << 0.20, -0.13, -0.30;  // 0.19, -0.1425, -0.30
    //     EE_Pose_RL_start << -0.18, 0.13, -0.30;  // -0.19, 0.1425, -0.30
    //     EE_Pose_RR_start << -0.18, -0.13, -0.30; // -0.19, -0.1425, -0.3

    //     EE_Pose_FL_final << 0.20, 0.13, -0.22;   // 0.19, 0.1425, -0.22
    //     EE_Pose_FR_final << 0.20, -0.13, -0.22;  // 0.19, -0.1425, -0.22
    //     EE_Pose_RL_final << -0.18, 0.13, -0.22;  // -0.19, 0.1425, -0.22
    //     EE_Pose_RR_final << -0.18, -0.13, -0.22; // -0.19, -0.1425, -0.22

    //     is_motion_started_ = true;
    // }

    // TrajectoryPoint fl_target = Sinusoidal_Task(motion_start_time_, motion_duration, EE_Pose_FL_start, EE_Pose_FL_final);
    // TrajectoryPoint fr_target = Sinusoidal_Task(motion_start_time_, motion_duration, EE_Pose_FR_start, EE_Pose_FR_final);
    // TrajectoryPoint rl_target = Sinusoidal_Task(motion_start_time_, motion_duration, EE_Pose_RL_start, EE_Pose_RL_final);
    // TrajectoryPoint rr_target = Sinusoidal_Task(motion_start_time_, motion_duration, EE_Pose_RR_start, EE_Pose_RR_final);

    // EE_Pose_FL_desired = fl_target.position;
    // EE_Vel_FL_desired = fl_target.velocity;

    // EE_Pose_FR_desired = fr_target.position;
    // EE_Vel_FR_desired = fr_target.velocity;

    // EE_Pose_RL_desired = rl_target.position;
    // EE_Vel_RL_desired = rl_target.velocity;

    // EE_Pose_RR_desired = rr_target.position;
    // EE_Vel_RR_desired = rr_target.velocity;

    // TaskSpacePDControl(80.0, 3.0);

    // Quintic_task (단일 이동)
    double motion_duration = 2.5;

    if (is_motion_started_)
    {
        // 작동 시간 판단 및 mode 변환 squating(앉기 / 일어나기)
        double t = (ros::Time::now() - motion_start_time_).toSec();

        if (t > motion_duration)
        {
            is_going_down_ = !is_going_down_;
            is_motion_started_ = false;
        }
    }

    else
    {
        motion_start_time_ = ros::Time::now();

        // 모션의 시작점 설정
        // EE_Pose_FL_start = EE_Pose_FL;
        // EE_Pose_FR_start = EE_Pose_FR;
        // EE_Pose_RL_start = EE_Pose_RL;
        // EE_Pose_RR_start = EE_Pose_RR;

        if (is_going_down_)
        {
            if (squat_count == 0)
            {
                std::array<Eigen::Vector3d, 4> ee_pose = KINE.Get_EE_Pose();
                EE_Pose_FL_start = ee_pose[0];
                EE_Pose_FR_start = ee_pose[1];
                EE_Pose_RL_start = ee_pose[2];
                EE_Pose_RR_start = ee_pose[3];
            }
            else
            {
                EE_Pose_FL_start << 0.20, 0.13, -0.38;
                EE_Pose_FR_start << 0.20, -0.13, -0.38;
                EE_Pose_RL_start << -0.18, 0.13, -0.38;
                EE_Pose_RR_start << -0.18, -0.13, -0.38;
            }

            // 모션의 최종 목표점 설정
            EE_Pose_FL_final << 0.20, 0.13, -0.25;
            EE_Pose_FR_final << 0.20, -0.13, -0.25;
            EE_Pose_RL_final << -0.18, 0.13, -0.25;
            EE_Pose_RR_final << -0.18, -0.13, -0.25;

            squat_count++;
        }
        else // 올라오기
        {
            EE_Pose_FL_start << 0.20, 0.13, -0.25;
            EE_Pose_FR_start << 0.20, -0.13, -0.25;
            EE_Pose_RL_start << -0.18, 0.13, -0.25;
            EE_Pose_RR_start << -0.18, -0.13, -0.25;

            // 모션의 최종 목표점 설정
            EE_Pose_FL_final << 0.20, 0.13, -0.38;
            EE_Pose_FR_final << 0.20, -0.13, -0.38;
            EE_Pose_RL_final << -0.18, 0.13, -0.38;
            EE_Pose_RR_final << -0.18, -0.13, -0.38;
        }

        is_motion_started_ = true;
    }

    TrajectoryPoint FL_target = TRAJ.Quintic_Task(motion_start_time_, motion_duration, EE_Pose_FL_start, EE_Pose_FL_final);
    TrajectoryPoint FR_target = TRAJ.Quintic_Task(motion_start_time_, motion_duration, EE_Pose_FR_start, EE_Pose_FR_final);
    TrajectoryPoint RL_target = TRAJ.Quintic_Task(motion_start_time_, motion_duration, EE_Pose_RL_start, EE_Pose_RL_final);
    TrajectoryPoint RR_target = TRAJ.Quintic_Task(motion_start_time_, motion_duration, EE_Pose_RR_start, EE_Pose_RR_final);

    // trajectory planning 설정 값, desired 값 설정
    EE_Pose_FL_desired = FL_target.position;
    EE_Pose_FR_desired = FR_target.position;
    EE_Pose_RL_desired = RL_target.position;
    EE_Pose_RR_desired = RR_target.position;

    // 초기 설정 속도
    EE_Vel_FL_desired = FL_target.velocity;
    EE_Vel_FR_desired = FR_target.velocity;
    EE_Vel_RL_desired = RL_target.velocity;
    EE_Vel_RR_desired = RR_target.velocity;

    TaskSpacePDControl(100.0, 10.0);
}

void go2_controller::SRBMControl()
{
    // CENT.Set_BodyState(gazebo_body_pos, gazebo_body_vel, gazebo_rpy, gazebo_rpy_dot, gazebo_quat);
    // std::array<Eigen::Vector3d, 4> feet_pos = KINE.Get_EE_Pose();
    // CENT.SetFootPosition_(feet_pos);

    // Eigen::Matrix<double, 6, 12> J = KINE.Get_Jacobian();
    // Eigen::MatrixXd F1_J_ = J.block<3,3>(0,0);
    // Eigen::MatrixXd F2_J_ = J.block<3,3>(0,3);
    // Eigen::MatrixXd H1_J_ = J.block<3,3>(0,6);
    // Eigen::MatrixXd H2_J_ = J.block<3,3>(0,9);
    // CENT.Set_LegJacobian(F1_J_, F2_J_, H1_J_, H2_J);

    // Body_Ref_ << 0, 0, 0.34, 0, 0, 0, gazebo_rpy(0), gazebo_rpy(1), gazebo_rpy(2),
    // CENT.Set_Reference(Body_Ref_);

    // 1. 로봇 몸통 상태 받아오기
    SRBM.SetRobotState(gazebo_body_pos, gazebo_body_vel, gazebo_quat, gazebo_rpy, gazebo_rpy_dot);
    std::array<Eigen::Vector3d, 4> feet_pos = KINE.Get_EE_Pose();
    SRBM.SetFootPosition(feet_pos);

    SRBM.Update_A_Matrix();
    Eigen::Quaterniond q_curr(gazebo_quat(3), gazebo_quat(0), gazebo_quat(1), gazebo_quat(2));
    Eigen::Matrix3d R_w2b = q_curr.toRotationMatrix().transpose();

    // 5. 계산 결과에 따른
    // double yaw = gazebo_rpy(2); // 현재 yaw (rad)

    // Eigen::Matrix3d R_yaw_only;
    // R_yaw_only << cos(yaw), -sin(yaw), 0,
    //     sin(yaw), cos(yaw), 0,
    //     0, 0, 1;
    Eigen::Matrix3d R_des_ = Eigen::Matrix3d::Identity();
    SRBM.Compute_b_Vector(gazebo_body_pos, R_des_);

    SRBM.Solve_QP();
    Force_ = SRBM.Get_Force();
    Force_FL = Force_.segment<3>(0);
    Force_FR = Force_.segment<3>(3);
    Force_RL = Force_.segment<3>(6);
    Force_RR = Force_.segment<3>(9);

    Eigen::Matrix<double, 6, 12> J = KINE.Get_Jacobian();
    std::array<Eigen::Vector3d, 4> Input_torque;

    Input_torque[0] = J.block<3, 3>(0, 0).transpose() * (-1) * Force_FL;
    Input_torque[1] = J.block<3, 3>(0, 3).transpose() * (-1) * Force_FR;
    Input_torque[2] = J.block<3, 3>(0, 6).transpose() * (-1) * Force_RL;
    Input_torque[3] = J.block<3, 3>(0, 9).transpose() * (-1) * Force_RR;

    torque_.segment<3>(0) = Input_torque[0];
    torque_.segment<3>(3) = Input_torque[1];
    torque_.segment<3>(6) = Input_torque[2];
    torque_.segment<3>(9) = Input_torque[3];

    std::cout << "SRBM_Force = " << std::endl
              << Force_ << std::endl;
    std::cout << "SRBM_Torque[0] = " << std::endl
              << Input_torque[0] << std::endl;
    std::cout << "SRBM_Torque[1] = " << std::endl
              << Input_torque[1] << std::endl;
    std::cout << "SRBM_Torque[2] = " << std::endl
              << Input_torque[2] << std::endl;
    std::cout << "SRBM_Torque[3] = " << std::endl
              << Input_torque[3] << std::endl;
}

void go2_controller::Run()
{
    ROS_INFO("Running the torque control loop .................");

    const ros::Duration control_period_(1.0 / 200.0); // 500hz

    ros::AsyncSpinner spinner(4); // 4자유도인거랑은 별개임 스레드 4개 사용해서 더 잘 처리한다는 뜻
    spinner.start();

    ros::Time start_time = ros::Time::now();
    ros::Time last_control_time = start_time;

    while (ros::ok())
    {
        ros::Time current_time = ros::Time::now();

        ros::Duration elapsed_time = current_time - last_control_time;

        if (elapsed_time >= control_period_)
        {
            last_control_time = current_time;

            Command(Recieved_Joint_State);

            ros::Duration sleep_time = control_period_ - elapsed_time;
            if (sleep_time > ros::Duration(0))
            {
                sleep_time.sleep();
            }
        }
    }
}

void go2_controller::PlotRun()
{
    ROS_INFO("Running the torque control loop .................");

    const ros::Duration control_period_(1.0 / 200.0); // 500hz

    ros::AsyncSpinner spinner(4); // 4자유도인거랑은 별개임 스레드 4개 사용해서 더 잘 처리한다는 뜻
    spinner.start();

    ros::Time start_time = ros::Time::now();
    ros::Time last_control_time = start_time;

    while (ros::ok())
    {
        ros::Time current_time = ros::Time::now();

        ros::Duration elapsed_time = current_time - last_control_time;

        if (elapsed_time >= control_period_)
        {
            last_control_time = current_time;

            DataStream();

            ros::Duration sleep_time = control_period_ - elapsed_time;
            if (sleep_time > ros::Duration(0))
            {
                sleep_time.sleep();
            }
        }
    }
}

void go2_controller::StateBodyCallback(const gazebo_msgs::ModelStates::ConstPtr &body)
{
    std::vector<std::string>::const_iterator iter = std::find(body->name.begin(), body->name.end(), "go2");
    if (iter != body->name.end())
    {
        int index = iter - body->name.begin();

        geometry_msgs::Point position = body->pose[index].position; // position
        geometry_msgs::Quaternion orientation = body->pose[index].orientation;
        geometry_msgs::Vector3 velocity = body->twist[index].linear;          // velocity
        geometry_msgs::Vector3 angular_velocity = body->twist[index].angular; // angular velocity

        tf::Quaternion q(
            body->pose[index].orientation.x, // (*body).pose와 동일
            body->pose[index].orientation.y,
            body->pose[index].orientation.z,
            body->pose[index].orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        gazebo_body_pos(X) = position.x;
        gazebo_body_pos(Y) = position.y;
        gazebo_body_pos(Z) = position.z;

        gazebo_body_vel(X) = velocity.x;
        gazebo_body_vel(Y) = velocity.y;
        gazebo_body_vel(Z) = velocity.z;

        gazebo_quat(0) = orientation.x;
        gazebo_quat(1) = orientation.y;
        gazebo_quat(2) = orientation.z;
        gazebo_quat(3) = orientation.w;

        gazebo_rpy(0) = roll;
        gazebo_rpy(1) = pitch;
        gazebo_rpy(2) = yaw;

        gazebo_rpy_dot(0) = angular_velocity.x;
        gazebo_rpy_dot(1) = angular_velocity.y;
        gazebo_rpy_dot(2) = angular_velocity.z;
    }
}

void go2_controller::StateLegCallback(const sensor_msgs::JointState &state)
{
    q_(0) = state.position[1];
    q_(1) = state.position[2];
    q_(2) = state.position[0];
    q_(3) = state.position[4];
    q_(4) = state.position[5];
    q_(5) = state.position[3];
    q_(6) = state.position[7];
    q_(7) = state.position[8];
    q_(8) = state.position[6];
    q_(9) = state.position[10];
    q_(10) = state.position[11];
    q_(11) = state.position[9];

    dq_(0) = state.velocity[1];
    dq_(1) = state.velocity[2];
    dq_(2) = state.velocity[0];
    dq_(3) = state.velocity[4];
    dq_(4) = state.velocity[5];
    dq_(5) = state.velocity[3];
    dq_(6) = state.velocity[7];
    dq_(7) = state.velocity[8];
    dq_(8) = state.velocity[6];
    dq_(9) = state.velocity[10];
    dq_(10) = state.velocity[11];
    dq_(11) = state.velocity[9];

    Recieved_Joint_State = true;
}

void go2_controller::SendCommandsToRobot()
{
    std_msgs::Float64MultiArray msg;

    for (size_t i = 0; i < 12; i++)
    {
        if (controlmode == INIT)
        {
            msg.data.push_back(0);
        }
        else
        {
            msg.data.push_back(torque_(i)); // torque_() : 토크값 저장.
        }
    }
    pub_leg_cmd_.publish(msg);

    msg.data.clear();
}

void go2_controller::DataStream()
{

    // PLOT---------------------------------------------------------------------------------------------------------------------------------------------------------

    std_msgs::Float64MultiArray TH_msg;

    // TH_msg 메세지의 형태로 data를 발행함.
    // ex) EE_Pose_FL은 x, y, z의 형태로 되어 있으니까, EE_Pose_FL

    TH_msg.data.push_back(Body_Pos(X));
    TH_msg.data.push_back(Body_Pos(Y));
    TH_msg.data.push_back(Body_Pos(Z));

    TH_msg.data.push_back(Force_FL(X));
    TH_msg.data.push_back(Force_FL(Y));
    TH_msg.data.push_back(Force_FL(Z));
    
    TH_msg.data.push_back(Force_FR(X));
    TH_msg.data.push_back(Force_FR(Y));
    TH_msg.data.push_back(Force_FR(Z));

    TH_msg.data.push_back(Force_RL(X));
    TH_msg.data.push_back(Force_RL(Y));
    TH_msg.data.push_back(Force_RL(Z));

    TH_msg.data.push_back(Force_RR(X));
    TH_msg.data.push_back(Force_RR(Y));
    TH_msg.data.push_back(Force_RR(Z));

    pub_TH_.publish(TH_msg);

    // PLOT---------------------------------------------------------------------------------------------------------------------------------------------------------

    // LOGDATA------------------------------------------------------------------------------------------------------------------------------------------------------

    // LOGDATA------------------------------------------------------------------------------------------------------------------------------------------------------
}