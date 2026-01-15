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
    sub_FL_contact_ = nh_.subscribe("/go2/FL_contactsensor_state", 1000, &go2_controller::FLcontactCallback, this);
    sub_FR_contact_ = nh_.subscribe("/go2/FR_contactsensor_state", 1000, &go2_controller::FRcontactCallback, this);
    sub_RL_contact_ = nh_.subscribe("/go2/RL_contactsensor_state", 1000, &go2_controller::RLcontactCallback, this);
    sub_RR_contact_ = nh_.subscribe("/go2/RR_contactsensor_state", 1000, &go2_controller::RRcontactCallback, this);

    // leg_state(다리상태)를 go2의 legstate를 구독함.
    /* StateLegCallback 함수는 코드상에서 직접 호출되는 부분이 없으며, ROS 시스템에 의해 특정 이벤트가 발생했을 때 자동으로 호출되도록 등록되어 있습니다. */

    // Advertiser (Publishers)
    /* pub_TH : plotjuggler로 Float의 형태로 본인의 상태를 발행하는거임.
    // pub_leg_cmd_ : gazebo로 로봇의 발위치 구독 */
    pub_leg_cmd_ = nh_.advertise<std_msgs::Float64MultiArray>(go2_topic_leg_command_, queue_size);
    pub_TH_ = nh_.advertise<std_msgs::Float64MultiArray>("TH", queue_size);

    controlmode = INIT;
    Recieved_Joint_State = false;

    q_.setZero(NUM_DOF);
    dq_.setZero(NUM_DOF);
    torque_.setZero(NUM_DOF);
    q_start.setZero(NUM_DOF);
    q_final.setZero(NUM_DOF);
    q_desired.setZero(NUM_DOF);
    contact_.setZero(NUM_LEG);

    M_Matrix.setZero(NUM_DOF, NUM_DOF);
    C_Matrix.setZero(NUM_DOF);
    G_Matrix.setZero(NUM_DOF);

    gazebo_body_pos.setZero(NUM_AXIS);
    gazebo_body_vel.setZero(NUM_AXIS);
    gazebo_quat.setZero(4);
    gazebo_rpy.setZero(NUM_AXIS);
    gazebo_rpy_dot.setZero(NUM_AXIS);

    Local_body_pos.setZero(NUM_AXIS);
    Local_body_vel.setZero(NUM_AXIS);
    Local_rpy_dot.setZero(NUM_AXIS);

    COM_Ref.setZero(NUM_LEG * NUM_AXIS);
    GRF.setZero(NUM_DOF);

    for (size_t i = 0; i < 4; ++i)
    {
        Foot_Pos[i].setZero(NUM_JOINT);
        Foot_Vel[i].setZero(NUM_JOINT);
        Foot_J[i].setZero(NUM_JOINT, NUM_JOINT);
        Torque[i].setZero(NUM_JOINT);
        Leg_Force[i].setZero(NUM_AXIS);
        Kp_Swing[i].setZero(NUM_AXIS, NUM_AXIS);
        Kd_Swing[i].setZero(NUM_AXIS, NUM_AXIS);
        Hor_Foot_pos[i].setZero(NUM_JOINT);
        Ver_Foot_pos[i].setZero(NUM_JOINT);
        Init_Foot_pos[i].setZero(NUM_JOINT);
        Trot_Gait[i].setZero(2 * T_TROT);
    }

    Start_Position.setZero(NUM_DOF);
    Homing_Position.setZero(NUM_DOF);

    Start_Position << 0, 1.20, -2.60, 0, 1.20, -2.60, 0, 1.20, -2.60, 0, 1.20, -2.60;
    Homing_Position << 0, 0.67, -1.40, 0, 0.67, -1.40, 0, 0.67, -1.40, 0, 0.67, -1.40;

    Contact_State.setZero(4);
    Trot_Pattern << 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0;

    // Swing 보행 검증: 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    // TROT 보행시 : 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0;
    // 자세안정화시 : 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    // FR : 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0;
    // RL : 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0;
}

void go2_controller::Command(bool flag)
{
    Set_Kinematics();
    // Set_Dynamics();
    Reference_Generator();

    if (Recieved_Joint_State)
    {
        switch (controlmode)
        {
        case INIT:
        {
            controlmode = HOMING;
            break;
        }
        case HOMING:
        {
            Homing();
            break;
        }
        case SQUATING:
        {
            Squating();
            break;
        }
        case POSTURE:
        {
            // Gait_Scheduler(); // 내 거
            Gait_Renewal(); // 성민이 형 버전
            Posture_Control();
        }
        }

        SendCommandsToRobot();
    }
}

void go2_controller::Homing() // 초기자세 설정 하는 코드
{
    /*  double dt = 1.0 / frequency;
        double t = tick * dt;        */

    if (Start_Flag == 0)
    {
        Start_Flag = 1;
        q_start = q_;
        q_final = Start_Position;
    }

    if (Start_Flag == 1)
    {
        double period = 500;
        q_desired = q_start + (q_final - q_start) * 0.5 * (1 - cos(3.14 * Init_Time / period));
        torque_ = 100.0 * (q_desired - q_) - 1.0 * dq_;

        if (Init_Time < period)
        {
            Init_Time++;
        }
        else if (Init_Time == period)
        {
            Init_Time = 0;
            Start_Flag = 2;
        }
    }

    if (Start_Flag == 2)
    {
        Start_Flag = 3;
        q_start = q_;
        q_final = Homing_Position;
    }

    if (Start_Flag == 3)
    {
        double period = 1000;
        q_desired = q_start + (q_final - q_start) * 0.5 * (1 - cos(3.14 * Init_Time / period));
        torque_ = 100.0 * (q_desired - q_) - 1.0 * dq_;

        if (Init_Time < period)
        {
            Init_Time++;
        }
        else if (Init_Time == period)
        {
            Init_Time = 1000;
            Start_Flag = 4;

            if (Start_Flag == 4)
            {
                Pos_Command[X] = 0.0; // Local_body_pos(X)
                Pos_Command[Y] = 0.0;               // Local_body_pos(Y)
                Pos_Command[Z] = Local_body_pos(Z);
                RPY_Command[ROLL] = gazebo_rpy(ROLL);
                RPY_Command[PITCH] = gazebo_rpy(PITCH);
                RPY_Command[YAW] = gazebo_rpy(YAW);

                Init_Foot_pos[FL] = PINO.GetPos(FL);
                Init_Foot_pos[FR] = PINO.GetPos(FR);
                Init_Foot_pos[RL] = PINO.GetPos(RL);
                Init_Foot_pos[RR] = PINO.GetPos(RR);

                Hor_Foot_pos[FL] = PINO.GetPos(FL);
                Hor_Foot_pos[FR] = PINO.GetPos(FR);
                Hor_Foot_pos[RL] = PINO.GetPos(RL);
                Hor_Foot_pos[RR] = PINO.GetPos(RR);

                Ver_Foot_pos[FL] = PINO.GetPos(FL);
                Ver_Foot_pos[FR] = PINO.GetPos(FR);
                Ver_Foot_pos[RL] = PINO.GetPos(RL);
                Ver_Foot_pos[RR] = PINO.GetPos(RR);

                controlmode = POSTURE;
            }
        }
    }

    // q_와 dq_는 계속 StateLegCallback 함수로 인해 실시간으로 값을 할당받는중임.
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

    //     EE_Pose_start[FL] << 0.20, 0.13, -0.30;   // 0.19, 0.1425, -0.30
    //     EE_Pose_start[FR] << 0.20, -0.13, -0.30;  // 0.19, -0.1425, -0.30
    //     EE_Pose_start[RL] << -0.18, 0.13, -0.30;  // -0.19, 0.1425, -0.30
    //     EE_Pose_start[RR] << -0.18, -0.13, -0.30; // -0.19, -0.1425, -0.3

    //     EE_Pose_start[FL] << 0.20, 0.13, -0.22;   // 0.19, 0.1425, -0.22
    //     EE_Pose_start[FR] << 0.20, -0.13, -0.22;  // 0.19, -0.1425, -0.22
    //     EE_Pose_start[RL] << -0.18, 0.13, -0.22;  // -0.19, 0.1425, -0.22
    //     EE_Pose_start[RR] << -0.18, -0.13, -0.22; // -0.19, -0.1425, -0.22

    //     is_motion_started_ = true;
    // }

    // TrajectoryPoint fl_target = Sinusoidal_Task(motion_start_time_, motion_duration, EE_Pose_start[FL], EE_Pose_start[FL]);
    // TrajectoryPoint fr_target = Sinusoidal_Task(motion_start_time_, motion_duration, EE_Pose_start[FR], EE_Pose_start[FR]);
    // TrajectoryPoint rl_target = Sinusoidal_Task(motion_start_time_, motion_duration, EE_Pose_start[RL], EE_Pose_start[RL]);
    // TrajectoryPoint rr_target = Sinusoidal_Task(motion_start_time_, motion_duration, EE_Pose_start[RR], EE_Pose_start[RR]);

    // EE_Pose_desired[FL] = fl_target.position;
    // EE_Vel_desired[FL] = fl_target.velocity;

    // EE_Pose_desired[FR] = fr_target.position;
    // EE_Vel_desired[FR] = fr_target.velocity;

    // EE_Pose_desired[RL] = rl_target.position;
    // EE_Vel_desired[RL] = rl_target.velocity;

    // EE_Pose_desired[RR] = rr_target.position;
    // EE_Vel_desired[RR] = rr_target.velocity;

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
        // EE_Pose_start[FL] = EE_Pose_FL;
        // EE_Pose_start[FR] = EE_Pose_FR;
        // EE_Pose_start[RL] = EE_Pose_RL;
        // EE_Pose_start[RR] = EE_Pose_RR;

        if (is_going_down_)
        {
            if (squat_count == 0)
            {
                std::array<Eigen::Vector3d, 4> ee_pose = KINE.Get_EE_Pose();
                EE_Pose_start[FL] = ee_pose[0];
                EE_Pose_start[FR] = ee_pose[1];
                EE_Pose_start[RL] = ee_pose[2];
                EE_Pose_start[RR] = ee_pose[3];
            }
            else
            {
                EE_Pose_start[FL] << 0.20, 0.13, -0.38;
                EE_Pose_start[FR] << 0.20, -0.13, -0.38;
                EE_Pose_start[RL] << -0.18, 0.13, -0.38;
                EE_Pose_start[RR] << -0.18, -0.13, -0.38;
            }

            // 모션의 최종 목표점 설정
            EE_Pose_final[FL] << 0.20, 0.13, -0.25;
            EE_Pose_final[FR] << 0.20, -0.13, -0.25;
            EE_Pose_final[RL] << -0.18, 0.13, -0.25;
            EE_Pose_final[RR] << -0.18, -0.13, -0.25;

            squat_count++;
        }
        else // 올라오기
        {
            EE_Pose_start[FL] << 0.20, 0.13, -0.25;
            EE_Pose_start[FR] << 0.20, -0.13, -0.25;
            EE_Pose_start[RL] << -0.18, 0.13, -0.25;
            EE_Pose_start[RR] << -0.18, -0.13, -0.25;

            // 모션의 최종 목표점 설정
            EE_Pose_start[FL] << 0.20, 0.13, -0.38;
            EE_Pose_start[FR] << 0.20, -0.13, -0.38;
            EE_Pose_start[RL] << -0.18, 0.13, -0.38;
            EE_Pose_start[RR] << -0.18, -0.13, -0.38;
        }

        is_motion_started_ = true;
    }

    QuinticTask FL_target = PLAN.Quintic_Task_rostime(motion_start_time_, motion_duration, EE_Pose_start[FL], EE_Pose_start[FL]);
    QuinticTask FR_target = PLAN.Quintic_Task_rostime(motion_start_time_, motion_duration, EE_Pose_start[FR], EE_Pose_start[FR]);
    QuinticTask RL_target = PLAN.Quintic_Task_rostime(motion_start_time_, motion_duration, EE_Pose_start[RL], EE_Pose_start[RL]);
    QuinticTask RR_target = PLAN.Quintic_Task_rostime(motion_start_time_, motion_duration, EE_Pose_start[RR], EE_Pose_start[RR]);

    // trajectory planning 설정 값, desired 값 설정
    EE_Pose_desired[FL] = FL_target.position;
    EE_Pose_desired[FR] = FR_target.position;
    EE_Pose_desired[RL] = RL_target.position;
    EE_Pose_desired[RR] = RR_target.position;

    // 초기 설정 속도
    EE_Vel_desired[FL] = FL_target.velocity;
    EE_Vel_desired[FR] = FR_target.velocity;
    EE_Vel_desired[RL] = RL_target.velocity;
    EE_Vel_desired[RR] = RR_target.velocity;

    TaskSpacePDControl(100.0, 100.0, 100.0, 10.0, 10.0, 10.0);
}

void go2_controller::SRBMControl()
{
    CENT.Set_RobotState(Local_body_pos, Local_body_vel, gazebo_quat, gazebo_rpy, Local_rpy_dot);
    CENT.Set_FootPosition(PINO.GetPos(FL), PINO.GetPos(FR), PINO.GetPos(RL), PINO.GetPos(RR));
    CENT.Set_RefGait(Trot_Gait);
    // CENT.Set_GaitPhase(Contact_State);
    CENT.Set_Reference(COM_Ref);
    CENT.Compute_A_Matrix();
    CENT.Compute_B_Vector();
    CENT.Set_CostFunction();
    CENT.Set_Constraint();
    CENT.Solve_QP();
}

void go2_controller::Reference_Generator()
{
    /* reference 를 한군데에다가 모아서 한 번에 전달. */
    COM_Ref << Pos_Command[X], Pos_Command[Y], Pos_Command[Z], Vel_Command[X], Vel_Command[Y], Vel_Command[Z],
        RPY_Command[X], RPY_Command[Y], RPY_Command[Z], ANG_Command[X], ANG_Command[Y], ANG_Command[Z];
}

void go2_controller::Gait_Scheduler()
{
    Switch_Time++;

    if (Switch_Time == T_STANCE) // 350 tick (0 ~ 349)
    {
        Gait_Switch += 1;
    }

    if (Switch_Time == T_TROT) // 500 tick 500 = 0 (350 ~ 499)
    {
        Switch_Time = 0;
        Gait_Switch = (Gait_Switch + 1) % 4;
    }

    // std::cout << "====== Switich_Time ====== : "  << Switch_Time << std::endl;
    Contact_State = Trot_Pattern.row(Gait_Switch);
}

void go2_controller::Gait_Renewal()
{
    GAIT.Gait_Update();

    for (int i = 0; i < NUM_LEG; i++)
    {
        Trot_Gait[i] = GAIT.Get_ReferenceGait(i);
    }
}

void go2_controller::StanceLeg_Control(int leg)
{
    GRF = CENT.Get_Force();

    Leg_Force[leg](X) = GRF(3 * leg);
    Leg_Force[leg](Y) = GRF(3 * leg + 1);
    Leg_Force[leg](Z) = GRF(3 * leg + 2);

    Posture_Torque[leg] = Foot_J[leg].transpose() * ((-1) * Leg_Force[leg]);

    Stance_Torque[leg](0) = Posture_Torque[leg](0);
    Stance_Torque[leg](1) = Posture_Torque[leg](1);
    Stance_Torque[leg](2) = Posture_Torque[leg](2);
}

void go2_controller::SwingLeg_Control(int leg)
{
    constexpr double step_height = 0.10;

    // 게인 설정
    Kp_Swing[leg].diagonal() << 3000.0, 3000.0, 3000.0; // 1800.0, 1800.0, 1500.0
    Kd_Swing[leg].diagonal() << 30.0, 30.0, 30.0;

    double horizontal_phase = static_cast<double>(Hor_Swing_Time[leg]) / static_cast<double>(T_SWING);
    double vertical_phase = static_cast<double>(Ver_Swing_Time[leg]) / static_cast<double>(T_SWING);

    EE_Pose_desired[leg](X) = Hor_Foot_pos[leg](X) + (Init_Foot_pos[leg](X) - Hor_Foot_pos[leg](X)) * 0.5 * (1 - cos(M_PI * horizontal_phase)); // 오차 보정
    EE_Pose_desired[leg](Y) = Hor_Foot_pos[leg](Y) + (Init_Foot_pos[leg](Y) - Hor_Foot_pos[leg](Y)) * 0.5 * (1 - cos(M_PI * horizontal_phase)); // 오차 보정
    EE_Pose_desired[leg](Z) = Init_Foot_pos[leg](Z) + step_height * 0.5 * (1 - cos(M_PI * vertical_phase));

    EE_Vel_desired[leg](X) = 0.0;
    EE_Vel_desired[leg](Y) = 0.0;
    EE_Vel_desired[leg](Z) = step_height * 0.5 * M_PI * sin(M_PI * vertical_phase);

    Leg_Force[leg] = Kp_Swing[leg] * (EE_Pose_desired[leg] - Foot_Pos[leg]) + Kd_Swing[leg] * (EE_Vel_desired[leg] - Foot_Vel[leg]);
    Swing_Torque[leg] = Foot_J[leg].transpose() * (Leg_Force[leg] + G_Matrix.segment<3>(3 * leg));

    Hor_Swing_Time[leg] += 1;
    Ver_Swing_Time[leg] += 1;
}

void go2_controller::Posture_Control()
{
    for (int leg = 0; leg < NUM_LEG; leg++)
    {
        if (Trot_Gait[leg](0) == STANCE) // Trot_Gait[leg](0) == STANCE , Contact_State[leg] == STANCE
        {
            StanceLeg_Control(leg);

            torque_(3 * leg) = Stance_Torque[leg](0);
            torque_(3 * leg + 1) = Stance_Torque[leg](1);
            torque_(3 * leg + 2) = Stance_Torque[leg](2);

            Hor_Foot_pos[leg] = PINO.GetPos(leg);
            Ver_Foot_pos[leg] = PINO.GetPos(leg);

            EE_Pose_start[leg] = Foot_Pos[leg];
            EE_Pose_start[leg] = Foot_Pos[leg];

            Hor_Swing_Time[leg] = 0;
            Ver_Swing_Time[leg] = 0;
        }

        else if (Trot_Gait[leg](0) == SWING) // Trot_Gait[leg](0) == SWING , Contact_State[leg] == SWING
        {
            SwingLeg_Control(leg);

            torque_(3 * leg) = Swing_Torque[leg](0);
            torque_(3 * leg + 1) = Swing_Torque[leg](1);
            torque_(3 * leg + 2) = Swing_Torque[leg](2);
        }
    }
}

void go2_controller::Run()
{
    ROS_INFO("Running the torque control loop .................");

    const ros::Duration control_period_(1.0 / FREQUENCY); // 200hz

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

void go2_controller::CentRun()
{
    ROS_INFO("Running the torque control loop .................");

    const ros::Duration control_period_(1.0 / FREQUENCY); // 200hz

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

            SRBMControl();

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

    const ros::Duration control_period_(1.0 / FREQUENCY); // 200hz

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

        Eigen::Quaterniond quat(orientation.w, orientation.x, orientation.y, orientation.z);
        quat.normalize();
        R_bw = quat.toRotationMatrix();
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

void go2_controller::FLcontactCallback(const gazebo_msgs::ContactsState::ConstPtr &msg)
{
    if (!msg->states.empty())
    {
        geometry_msgs::Vector3 force = msg->states[0].total_wrench.force;

        ContactSensorCkFlag[FL] = 1;
        contact_[FL] = true;
        Contact[FL] = 1;

        ft_sensor_force[FL] << force.x, force.y, force.z;
    }
    else
    {
        contact_[FL] = false;
        Contact[FL] = 0;
        ft_sensor_force[FL].setZero();
    }
}

void go2_controller::FRcontactCallback(const gazebo_msgs::ContactsState::ConstPtr &msg)
{
    geometry_msgs::Vector3 force = msg->states[0].total_wrench.force;
    if (!msg->states.empty())
    {
        ContactSensorCkFlag[FR] = 1;
        contact_[FR] = true;
        Contact[FR] = 1;
        ft_sensor_force[FR] << force.x, force.y, force.z;
    }
    else
    {
        contact_[FR] = false;
        Contact[FR] = 0;
        ft_sensor_force[FR].setZero();
    }
}

void go2_controller::RLcontactCallback(const gazebo_msgs::ContactsState::ConstPtr &msg)
{
    geometry_msgs::Vector3 force = msg->states[0].total_wrench.force;
    if (!msg->states.empty())
    {
        ContactSensorCkFlag[RL] = 1;
        contact_[RL] = true;
        Contact[RL] = 1;
        ft_sensor_force[RL] << force.x, force.y, force.z;
    }
    else
    {
        contact_[RL] = false;
        Contact[RL] = 0;
        ft_sensor_force[RL].setZero();
    }
}

void go2_controller::RRcontactCallback(const gazebo_msgs::ContactsState::ConstPtr &msg)
{
    geometry_msgs::Vector3 force = msg->states[0].total_wrench.force;
    if (!msg->states.empty())
    {
        ContactSensorCkFlag[RR] = 1;
        contact_[RR] = true;
        Contact[RR] = 1;
        ft_sensor_force[RR] << force.x, force.y, force.z;
    }
    else
    {
        contact_[RR] = false;
        Contact[RR] = 0;
        ft_sensor_force[RR].setZero();
    }
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

    Eigen::Vector3d Ref_Pos_ = CENT.Get_RefPos();
    Eigen::Vector3d Err_Pos_ = CENT.Get_Error_Pose();
    Eigen::Vector3d Err_Ori_ = CENT.Get_Error_R();
    Eigen::Vector3d EPOS = (Ref_Pos_ - Err_Pos_);

    // SRBM 추출 힘

    TH_msg.data.push_back(EE_Pose_desired[FL](X));
    TH_msg.data.push_back(Foot_Pos[FL](X));
    TH_msg.data.push_back(EE_Pose_desired[FL](Y));
    TH_msg.data.push_back(Foot_Pos[FL](Y));
    TH_msg.data.push_back(EE_Pose_desired[FL](Z));
    TH_msg.data.push_back(Foot_Pos[FL](Z));

    TH_msg.data.push_back(EE_Pose_desired[FR](X));
    TH_msg.data.push_back(Foot_Pos[FR](X));
    TH_msg.data.push_back(EE_Pose_desired[FR](Y));
    TH_msg.data.push_back(Foot_Pos[FR](Y));
    TH_msg.data.push_back(EE_Pose_desired[FR](Z));
    TH_msg.data.push_back(Foot_Pos[FR](Z));

    TH_msg.data.push_back(EE_Pose_desired[RL](X));
    TH_msg.data.push_back(Foot_Pos[RL](X));
    TH_msg.data.push_back(EE_Pose_desired[RL](Y));
    TH_msg.data.push_back(Foot_Pos[RL](Y));
    TH_msg.data.push_back(EE_Pose_desired[RL](Z));
    TH_msg.data.push_back(Foot_Pos[RL](Z));

    TH_msg.data.push_back(EE_Pose_desired[RR](X));
    TH_msg.data.push_back(Foot_Pos[RR](X));
    TH_msg.data.push_back(EE_Pose_desired[RR](Y));
    TH_msg.data.push_back(Foot_Pos[RR](Y));
    TH_msg.data.push_back(EE_Pose_desired[RR](Z));
    TH_msg.data.push_back(Foot_Pos[RR](Z));

    TH_msg.data.push_back(Ref_Pos_(X));
    TH_msg.data.push_back(Ref_Pos_(Y));
    TH_msg.data.push_back(Ref_Pos_(Z));

    TH_msg.data.push_back(Err_Pos_(X));
    TH_msg.data.push_back(Err_Pos_(Y));
    TH_msg.data.push_back(Err_Pos_(Z));

    TH_msg.data.push_back(RPY_Command[ROLL]);
    TH_msg.data.push_back(RPY_Command[PITCH]);
    TH_msg.data.push_back(RPY_Command[YAW]);

    TH_msg.data.push_back(gazebo_rpy(ROLL));
    TH_msg.data.push_back(gazebo_rpy(PITCH));
    TH_msg.data.push_back(gazebo_rpy(YAW));

    TH_msg.data.push_back(Local_body_pos(X));
    TH_msg.data.push_back(Local_body_pos(Y));
    TH_msg.data.push_back(Local_body_pos(Z));

    // TH_msg.data.push_back(count);

    pub_TH_.publish(TH_msg);

    // PLOT---------------------------------------------------------------------------------------------------------------------------------------------------------

    // LOGDATA------------------------------------------------------------------------------------------------------------------------------------------------------

    // LOGDATA------------------------------------------------------------------------------------------------------------------------------------------------------
}

void go2_controller::Set_FK_Kinematics()
{
    KINE.Forward_Kinematics(q_, dq_);
    KINE.Jacobian(q_);

    ee_pose = KINE.Get_EE_Pose();
    jacobian = KINE.Get_Jacobian();

    ee_vel[FL] = jacobian.block<3, 3>(0, 0) * dq_.segment<3>(0);
    ee_vel[FR] = jacobian.block<3, 3>(0, 3) * dq_.segment<3>(3);
    ee_vel[RL] = jacobian.block<3, 3>(0, 6) * dq_.segment<3>(6);
    ee_vel[RR] = jacobian.block<3, 3>(0, 9) * dq_.segment<3>(9);

    // R_bw = CENT.Get_R_BodyToWorld();
}

void go2_controller::Set_Kinematics()
{
    PINO.SetRobotParameter(q_, dq_);

    for (size_t i = 0; i < NUM_LEG; i++)
    {
        PINO.SetKinematics(i);
        Foot_J[i] = PINO.GetJacobian(i);
        Foot_Pos[i] = PINO.GetPos(i);
        Foot_Vel[i] = PINO.GetVel(i);
    }

    int stance_count = Contact[FL] + Contact[FR] + Contact[RL] + Contact[RR];

    if (stance_count > 0)
    {
        double inv_stance_count = 1.0 / (double)stance_count;
        Local_body_pos(X) = -(Foot_Pos[FL](X) * Contact[FL] + Foot_Pos[FR](X) * Contact[FR] +
                              Foot_Pos[RL](X) * Contact[RL] + Foot_Pos[RR](X) * Contact[RR]) *
                            inv_stance_count;

        Local_body_pos(Y) = -(Foot_Pos[FL](Y) * Contact[FL] + Foot_Pos[FR](Y) * Contact[FR] +
                              Foot_Pos[RL](Y) * Contact[RL] + Foot_Pos[RR](Y) * Contact[RR]) *
                            inv_stance_count;

        Local_body_pos(Z) = -(Foot_Pos[FL](Z) * Contact[FL] + Foot_Pos[FR](Z) * Contact[FR] +
                              Foot_Pos[RL](Z) * Contact[RL] + Foot_Pos[RR](Z) * Contact[RR]) *
                            inv_stance_count;
    }

    Local_body_pos(X) = Local_body_pos(X);
    Local_body_pos(Y) = Local_body_pos(Y);
    Local_body_pos(Z) = gazebo_body_pos(Z);

    Local_body_vel = R_bw.transpose() * gazebo_body_vel;
    Local_rpy_dot = R_bw.transpose() * gazebo_rpy_dot;
}

void go2_controller::Set_Dynamics()
{
    PINO.SetGravity();
    G_Matrix = PINO.GetGravityCompensation();
}