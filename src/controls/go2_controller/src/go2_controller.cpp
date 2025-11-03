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
    sub_leg_state_ = nh_.subscribe(go2_topic_leg_state_, queue_size, &go2_controller::StateLegCallback, this, ros::TransportHints().reliable().tcpNoDelay());
    // leg_state(다리상태)를 go2의 legstate를 구독함.
    /* StateLegCallback 함수는 코드상에서 직접 호출되는 부분이 없으며, ROS 시스템에 의해 특정 이벤트가 발생했을 때 자동으로 호출되도록 등록되어 있습니다. */

    // pub_TH : plotjuggler로 Float의 형태로 본인의 상태를 발행하는거임.
    pub_TH_ = nh_.advertise<std_msgs::Float64MultiArray>("TH", queue_size);
    pub_leg_cmd_ = nh_.advertise<std_msgs::Float64MultiArray>(go2_topic_leg_command_, queue_size);

    controlmode = INIT;
    Recieved_Joint_State = false;

    q_.setZero(12);
    dq_.setZero(12);
    torque_.setZero(12);

    // Joint Space PD 제어(Homing) : 0, 0.67, -1.40, 0, 0.67, -1.40, 0, 0.67, -1.40, 0, 0.67, -1.40
}

void go2_controller::Command(bool flag)
{
    if (Recieved_Joint_State)
    {
        Forward_Kinematics(q_, dq_);
        Jacobians_URDF(q_);
        // Forward_Kinematics_ME(q_, dq_);
        // Create_Jacobian(q_);

        const double squat_duration = 2.5;

        switch (controlmode)
        {
        case INIT:
        {
            // Homing 시작 시 초기 상태 저장
            q_current = q_;
            Homing_Time = ros::Time::now();
            controlmode = HOMING;
            break;
        }
        case HOMING:
        {
            Homing();
            // torque_(0) = -50;

            if ((ros::Time::now() - Homing_Time).toSec() >= 2.0)
            {
                controlmode = SQUAT_START;
            }
            break;
        }
        case SQUAT_START: // [수정됨] SQUATING_INIT 대신 사용
        {
            // 이 케이스는 단 1틱만 실행됩니다.
            motion_start_time_ = ros::Time::now();
            squat_count = 0;

            // [Jerk 해결] Homing 직후의 '실제' 발 위치를 스쿼트의 '시작' 위치로 설정
            EE_Pose_FL_start = EE_Pose_FL;
            EE_Pose_FR_start = EE_Pose_FR;
            EE_Pose_RL_start = EE_Pose_RL;
            EE_Pose_RR_start = EE_Pose_RR;

            // 첫 번째 목표 지점 (내려가기) 설정
            EE_Pose_FL_final << 0.20, 0.13, -0.38; // 더 깊은 스쿼트
            EE_Pose_FR_final << 0.20, -0.13, -0.38;
            EE_Pose_RL_final << -0.18, 0.13, -0.38;
            EE_Pose_RR_final << -0.18, -0.13, -0.38;

            controlmode = SQUAT_DOWN; // 다음 틱부터 SQUAT_DOWN 실행
            
            // [Graph Drop 해결] 첫 틱의 궤적을 즉시 계산하고 제어기에 전달
            Squating(); 
            break;
        }
        case SQUAT_DOWN:
        {
            Squating(); // "내려가기" 궤적 계산 및 제어

            // 궤적 종료 검사
            if ((ros::Time::now() - motion_start_time_).toSec() > squat_duration)
            {
                // [재초기화] 다음 동작(올라가기)을 위해 상태 재설정
                motion_start_time_ = ros::Time::now(); // 타이머 리셋
                
                EE_Pose_FL_start = EE_Pose_FL_final;
                EE_Pose_FR_start = EE_Pose_FR_final;
                EE_Pose_RL_start = EE_Pose_RL_final;
                EE_Pose_RR_start = EE_Pose_RR_final;

                // 새 목표점(올라가기) 설정 (Homing 자세)
                EE_Pose_FL_final << 0.20, 0.13, -0.25;
                EE_Pose_FR_final << 0.20, -0.13, -0.25;
                EE_Pose_RL_final << -0.18, 0.13, -0.25;
                EE_Pose_RR_final << -0.18, -0.13, -0.25;

                controlmode = SQUAT_UP; // 상태 변경
            }
            break;
        }
        case SQUAT_UP:
        {
            Squating(); // "올라가기" 궤적 계산 및 제어

            // 궤적 종료 검사
            if ((ros::Time::now() - motion_start_time_).toSec() > squat_duration)
            {
                // [재초기화] 다음 동작(내려가기)을 위해 상태 재설정
                motion_start_time_ = ros::Time::now(); // 타이머 리셋
                squat_count++;

                EE_Pose_FL_start = EE_Pose_FL_final;
                EE_Pose_FR_start = EE_Pose_FR_final;
                EE_Pose_RL_start = EE_Pose_RL_final;
                EE_Pose_RR_start = EE_Pose_RR_final;

                // 새 목표점(내려가기) 설정
                EE_Pose_FL_final << 0.20, 0.13, -0.38;
                EE_Pose_FR_final << 0.20, -0.13, -0.38;
                EE_Pose_RL_final << -0.18, 0.13, -0.38;
                EE_Pose_RR_final << -0.18, -0.13, -0.38;

                controlmode = SQUAT_DOWN; // 상태 변경 (무한 반복)
            }
            break;
        }
        // case SQUATING:
        // {
        //     Squating();
        //     break;
        // }
        }

        SendCommandsToRobot();
    }
}

void go2_controller::Homing() // 초기자세 설정 하는 코드
{
    // 성민이형 스타일
    // if(Homing_Time == 1000)
    // {
    //     Homing_Time = 1000;
    // }
    // else if (Homing_Time < 1000)
    // {
    //     Homing_Time++;
    // }

    // // quintic trajectory
    ros::Time Current_Time = ros::Time::now();
    double t = (Current_Time - Homing_Time).toSec(); // toSec() 적으세요
    double T = 2.0;

    q_final << 0, 0.67, -1.40, 0, 0.67, -1.40, 0, 0.67, -1.40, 0, 0.67, -1.40;

    Eigen::Matrix<double, 6, 6> M; // double 요소의 6x6 행렬 선언
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;

    M << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 2, 0, 0, 0,
        1, T, T2, T3, T4, T5,
        0, 1, 2 * T, 3 * T2, 4 * T3, 5 * T4,
        0, 0, 2, 6 * T, 12 * T2, 20 * T3;

    Eigen::Matrix<double, 6, 6> M_inv = M.inverse(); // M의 역행렬을 구함

    Eigen::Matrix<double, 6, 12> B;   // 12개의 관절들의 초기 (위치, 속도, 가속도), 최종 (위치, 속도, 가속도) 구하기
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
    }

    double K_p = 30.0;
    double K_d = 1.0;

    torque_ = K_p * (q_desired - q_) - K_d * dq_; // q_와 dq_는 계속 StateLegCallback 함수로 인해 실시간으로 값을 할당받는중임.
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
    // [수정됨] 이 함수는 'SQUAT_START', 'SQUAT_DOWN', 'SQUAT_UP' 상태에서
    // 매 틱 호출됩니다. 모든 상태 관리 로직을 제거합니다.

    double motion_duration = 2.5;

    // Command() 함수에서 설정한 _start, _final 값에 따라 궤적 생성
    EE_Pose_FL_desired = Quintic_Task(motion_start_time_, motion_duration, EE_Pose_FL_start, EE_Pose_FL_final);
    EE_Pose_FR_desired = Quintic_Task(motion_start_time_, motion_duration, EE_Pose_FR_start, EE_Pose_FR_final);
    EE_Pose_RL_desired = Quintic_Task(motion_start_time_, motion_duration, EE_Pose_RL_start, EE_Pose_RL_final);
    EE_Pose_RR_desired = Quintic_Task(motion_start_time_, motion_duration, EE_Pose_RR_start, EE_Pose_RR_final);

    // 초기 설정 속도
    EE_Vel_FL_desired.setZero();
    EE_Vel_FR_desired.setZero();
    EE_Vel_RL_desired.setZero();
    EE_Vel_RR_desired.setZero();

    // [중요] TaskSpacePDControl은 중력 보상을 포함하지 않습니다.
    // Homing (Joint PD)은 오차로 중력을 버텼지만, Task PD는 오차가 0이면
    // 중력을 버틸 토크를 생성하지 않아 무너집니다.
    // 우선은 TaskSpacePDControl을 호출합니다.
    TaskSpacePDControl(100.0, 3.0); 

    // 만약 이래도 무너진다면, TaskSpacePDControl.cpp에 
    // 중력 보상 토크(G(q))를 추가해야 합니다. (이것이 근본적인 해결책입니다)
}

void go2_controller::Run()
{
    ROS_INFO("Running the torque control loop .................");

    const ros::Duration control_period_(1.0 / 500.0); // 500hz

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

    const ros::Duration control_period_(1.0 / 500.0); // 500hz

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
    TH_msg.data.push_back(EE_Pose_FL(X));
    TH_msg.data.push_back(EE_Pose_FL(Y));
    TH_msg.data.push_back(EE_Pose_FL(Z));

    TH_msg.data.push_back(EE_Pose_FL_desired(X));
    TH_msg.data.push_back(EE_Pose_FL_desired(Y));
    TH_msg.data.push_back(EE_Pose_FL_desired(Z));

    pub_TH_.publish(TH_msg);

    // PLOT---------------------------------------------------------------------------------------------------------------------------------------------------------

    // LOGDATA------------------------------------------------------------------------------------------------------------------------------------------------------

    // LOGDATA------------------------------------------------------------------------------------------------------------------------------------------------------
}