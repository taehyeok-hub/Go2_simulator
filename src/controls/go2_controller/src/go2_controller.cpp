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
    /* pub_TH : plotjuggler로 Float의 형태로 본인의 상태를 발행하는거임.
    // pub_leg_cmd_ : gazebo로 로봇의 발위치 구독 */
    pub_TH_ = nh_.advertise<std_msgs::Float64MultiArray>("TH", queue_size);
    pub_leg_cmd_ = nh_.advertise<std_msgs::Float64MultiArray>(go2_topic_leg_command_, queue_size);

    controlmode = INIT;
    Recieved_Joint_State = false;

    q_.setZero(12);
    dq_.setZero(12);
    torque_.setZero(12);
    q_start.setZero(12);
    q_final.setZero(12);
    q_desired.setZero(12);

    Start_Position.setZero(12);
    Homing_Position.setZero(12);

    gazebo_body_pos.setZero(NUM_AXIS);
    gazebo_body_vel.setZero(NUM_AXIS);
    gazebo_quat.setZero(4);
    gazebo_rpy.setZero(NUM_AXIS);
    gazebo_rpy_dot.setZero(NUM_AXIS);

    COM_Ref.setZero(NUM_LEG * NUM_AXIS);

    for (size_t i = 0; i < 4; ++i)
    {
        Foot_Pos[i].setZero(NUM_JOINT);
        Foot_Vel[i].setZero(NUM_JOINT);
        Foot_J[i].setZero(NUM_JOINT, NUM_JOINT);
        Torque[i].setZero(NUM_JOINT);
    }

    Start_Position << 0, 1.20, -2.60, 0, 1.20, -2.60, 0, 1.20, -2.60, 0, 1.20, -2.60;
    Homing_Position << 0, 0.67, -1.40, 0, 0.67, -1.40, 0, 0.67, -1.40, 0, 0.67, -1.40;
}

void go2_controller::Command(bool flag)
{
    KINE.Forward_Kinematics(q_, dq_);
    KINE.Jacobian(q_);
    Reference_Generator();
    Set_Kinematics();

    if (Recieved_Joint_State)
    {
        // std::cout << "몸통 좌표 : x = " << gazebo_body_pos(0) << " y = " << gazebo_body_pos(1) << " z = " << gazebo_body_pos(2) << std::endl;
        // Forward_Kinematics_ME(q_, dq_);
        // Create_Jacobian(q_);

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
            Posture_Control();
            break;
        }
        // case SRBM_CONTROL:
        // {
        //     Body_Ref = gazebo_body_pos;
        //     break;
        // }
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
        double period = 250;
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
        double period = 750;
        q_desired = q_start + (q_final - q_start) * 0.5 * (1 - cos(3.14 * Init_Time / period));
        torque_ = 100.0 * (q_desired - q_) - 1.0 * dq_;

        if (Init_Time < period)
        {
            Init_Time++;
        }
        else if (Init_Time == period)
        {
            Init_Time = 1000;
            Start_Flag = 0;
            Pos_Command[X] = gazebo_body_pos(X);
            Pos_Command[Y] = gazebo_body_pos(Y);
            Pos_Command[Z] = gazebo_body_pos(Z);
            RPY_Command[ROLL] = 0.0;
            RPY_Command[PITCH] = 0.0;
            RPY_Command[YAW] = 0.0;
            controlmode = POSTURE;
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

    TrajectoryPoint FL_target = PLAN.Quintic_Task(motion_start_time_, motion_duration, EE_Pose_FL_start, EE_Pose_FL_final);
    TrajectoryPoint FR_target = PLAN.Quintic_Task(motion_start_time_, motion_duration, EE_Pose_FR_start, EE_Pose_FR_final);
    TrajectoryPoint RL_target = PLAN.Quintic_Task(motion_start_time_, motion_duration, EE_Pose_RL_start, EE_Pose_RL_final);
    TrajectoryPoint RR_target = PLAN.Quintic_Task(motion_start_time_, motion_duration, EE_Pose_RR_start, EE_Pose_RR_final);

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
    std::array<Eigen::Vector3d, 4> EE_Pose_ = KINE.Get_EE_Pose();
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    // 1. gazebo에서 world 좌표계 기준 com 받아오기, CENT 저장
    CENT.Set_RobotState(gazebo_body_pos, gazebo_body_vel, gazebo_quat, gazebo_rpy, gazebo_rpy_dot);
    CENT.Set_FKFootPosition(EE_Pose_);
    // CENT.Set_FootPosition(PINO.GetPos(FL), PINO.GetPos(FR), PINO.GetPos(RL), PINO.GetPos(RR));
    CENT.Set_Reference(COM_Ref);
    CENT.Compute_A_Matrix();
    CENT.Compute_B_Vector();
    CENT.Set_CostFunction();
    CENT.Solve_QP();   
}

void go2_controller::Posture_Control()
{
    GRF = CENT.Get_Force(); // 지면반발력
    Eigen::Matrix<double, 6, 12> J = KINE.Get_Jacobian();

    for (int i = 0; i < 4; i++)
    {
        // torque_.segment<3>(3 * i) = Foot_J[i].transpose() * (-1) * GRF.segment<3>(3 * i); (Pinocchio를 사용했을 경우)
        torque_.segment<3>(3 * i) = J.block<3,3>(0, 3 * i).transpose() * (-1) * GRF.segment<3>(3 * i); // (FK를 사용했을 경우)
    }
    std::cout << "=== Torque === \n" << torque_.transpose() << std::endl;
}

void go2_controller::Reference_Generator() 
{
    /* reference 를 한군데에다가 모아서 한 번에 전달. */ 
    COM_Ref << Pos_Command[X], Pos_Command[Y], Pos_Command[Z], Vel_Command[X], Vel_Command[Y], Vel_Command[Z],
               RPY_Command[X], RPY_Command[Y], RPY_Command[Z], ANG_Command[X], ANG_Command[Y], ANG_Command[Z]; 
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

void go2_controller::CentRun()
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

    Eigen::Vector3d Err_Pos_ = CENT.Get_Error_Pose();
    Eigen::Vector3d Err_Ori_ = CENT.Get_Error_R();

    // SRBM 추출 힘
    TH_msg.data.push_back(Pos_Command[X]);
    TH_msg.data.push_back(Pos_Command[Y]);
    TH_msg.data.push_back(Pos_Command[Z]);

    TH_msg.data.push_back(gazebo_body_pos(X));
    TH_msg.data.push_back(gazebo_body_pos(Y));
    TH_msg.data.push_back(gazebo_body_pos(Z));

    count += 1;

    TH_msg.data.push_back(count);

    pub_TH_.publish(TH_msg);

    // PLOT---------------------------------------------------------------------------------------------------------------------------------------------------------

    // LOGDATA------------------------------------------------------------------------------------------------------------------------------------------------------

    // LOGDATA------------------------------------------------------------------------------------------------------------------------------------------------------
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

    // int stance_count = Contact[FL] + Contact[FR] + Contact[RL] + Contact[RR];

    // if (stance_count > 0)
    // {
    //     double inv_stance_count = 1.0 / (double)stance_count;
    //     Local_body_pos(X) = -(Foot_Pos[FL](X) * Contact[FL] + Foot_Pos[FR](X) * Contact[FR] +
    //                           Foot_Pos[RL](X) * Contact[RL] + Foot_Pos[RR](X) * Contact[RR]) *
    //                         inv_stance_count;

    //     Local_body_pos(Y) = -(Foot_Pos[FL](Y) * Contact[FL] + Foot_Pos[FR](Y) * Contact[FR] +
    //                           Foot_Pos[RL](Y) * Contact[RL] + Foot_Pos[RR](Y) * Contact[RR]) *
    //                         inv_stance_count;

    //     Local_body_pos(Z) = -(Foot_Pos[FL](Z) * Contact[FL] + Foot_Pos[FR](Z) * Contact[FR] +
    //                           Foot_Pos[RL](Z) * Contact[RL] + Foot_Pos[RR](Z) * Contact[RR]) *
    //                         inv_stance_count;
    // }

    // Local_body_pos(Z) = Body_Height;

    // Local_body_vel = Rz_.transpose() * gazebo_body_vel;
    // Local_rpy_dot = Rz_.transpose() * gazebo_rpy_dot;
}