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

    Start_Position << 0, 1.20, -2.60, 0, 1.20, -2.60, 0, 1.20, -2.60, 0, 1.20, -2.60;
    Homing_Position << 0, 0.67, -1.40, 0, 0.67, -1.40, 0, 0.67, -1.40, 0, 0.67, -1.40;
}

void go2_controller::Command(bool flag)
{
    KINE.Forward_Kinematics(q_, dq_);
    KINE.Jacobian(q_);

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
            Body_Ref = gazebo_body_pos;
            // double alpha = 0.02;
            // Body_Pos.setZero();
            // Body_Rot.setZero();
            // Body_Pos = (1 - alpha) * Body_Pos + alpha * gazebo_body_pos;
            // Body_Rot = SRBM.RPYRotationMatrix(gazebo_rpy(0), gazebo_rpy(1), gazebo_rpy(2));
            // Body_Pos_ = gazebo_body_pos;
            break;
        }
        }

        SendCommandsToRobot();
    }
}

void go2_controller::Homing() // 초기자세 설정 하는 코드
{
    double t = (Current_Time - Moving_Time).toSec();

    if (Start_Flag == 0) // Homing 시작 시 초기 상태 저장
    {
        Start_Flag = 1;
        q_start = q_;
        q_final = Start_Position;
        Current_Time = ros::Time::now();
    }

    if (Start_Flag == 1)
    {
        Start_Flag = 2;

        Motion_Time = 1.0;
        double t = Trajectory::Set_TimeVariable(Current_Time, Motion_Time); // static 메서드
        q_desired = PLAN.Quintic_Joint(t, Motion_Time, q_start, q_final);

        torque_ = 100.0 * (q_desired - q_) - 1.0 * dq_;
    }

    if (Start_Flag == 3)
    {
        Start_Flag = 4;
        q_start = q_; // q_start = q_final; 로 해도 상관 X
        q_final = Homing_Position;
        Current_Time = ros::Time::now();
    }

    if (Start_Flag == 4)
    {
        double t = Trajectory::Set_TimeVariable(Current_Time, Motion_Time);
        Motion_Time = 3.0;
        q_desired = PLAN.Quintic_Joint(t, Motion_Time, q_start, q_final);

        torque_ = 100.0 * (q_desired - q_) - 1.0 * dq_;
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
    // Eigen::Matrix3d Des_Rot = Eigen::Matrix3d::Identity(); // Orientation (Identity)

    // // 2. FK 및 Jacobian 업데이트 (현재 상태)
    // std::array<Eigen::Vector3d, 4> EE_Pose_Body = KINE.Get_EE_Pose(); // Body Frame 발 위치

    // // 3. Centroidal Dynamics 업데이트
    // // 로봇 상태 설정 (World Frame States)
    // CENT.Set_RobotState(gazebo_body_pos, gazebo_body_vel, gazebo_quat, gazebo_rpy, gazebo_rpy_dot);

    // // 발 위치 설정 (Body Frame 값을 넘겨주면 내부에서 World Frame으로 변환됨)
    // CENT.Set_FootPosition(EE_Pose_Body);

    // // A Matrix 계산 (World Frame Lever arms)
    // CENT.Compute_A_Matrix();

    // // B Vector 계산 (Desired Dynamics in World Frame)
    // CENT.Compute_B_Vector(Body_Ref_, Des_Rot);

    // // QP 풀기
    // // CENT.Set_Constraint(200.0); // 필요시 최대 힘 조절
    // CENT.Solve_QP();

    // // 4. 토크 계산
    // // QP 결과: F_world (World Frame Force)
    // Eigen::VectorXd F_world = CENT.Get_Force_World();

    // // Jacobian: J_body (Body Frame Velocity -> Joint Velocity 관계)
    // Eigen::Matrix<double, 6, 12> J_body = KINE.Get_Jacobian();

    // // Rotation Matrix (Body to World)
    // Eigen::Matrix3d R_bw = CENT.Get_R_body_to_world();
    // Eigen::Matrix3d R_wb = R_bw.transpose(); // World to Body

    // for (int i = 0; i < 4; i++)
    // {
    //     Eigen::Vector3d F_leg_world = F_world.segment<3>(3 * i);

    //     // [중요] World Force를 Body Force로 변환
    //     // 로봇이 지면을 미는 힘(QP 결과) -> Body Frame으로 표현
    //     Eigen::Vector3d F_leg_body = R_wb * F_leg_world;

    //     // [중요] Torque = J^T * F_ext
    //     // F_ext는 로봇 발끝에 작용하는 힘 (지면 반력).
    //     // 부호: (-1) 제거. 지면 반력 방향 그대로 토크로 변환되어야 중력을 이김.
    //     torque_.segment<3>(3 * i) = J_body.block<3, 3>(0, 3 * i).transpose() * (-1) * F_leg_body;
    // }

    std::array<Eigen::Vector3d, 4> EE_Pose_ = KINE.Get_EE_Pose();
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    // 1. gazebo에서 world 좌표계 기준 com 받아오기, CENT 저장
    CENT.Set_RobotState(gazebo_body_pos, gazebo_body_vel, gazebo_quat, gazebo_rpy, gazebo_rpy_dot);
    CENT.Set_FKFootPosition(EE_Pose_);
    CENT.Compute_A_Matrix();
    CENT.Compute_B_Vector(Body_Ref, I);

    // CENT.Set_Reference();
    CENT.Solve_QP();
    Force = CENT.Get_Force();
    Eigen::Matrix<double, 6, 12> J = KINE.Get_Jacobian();

    for (int i = 0; i < 4; i++)
    {
        torque_.segment<3>(3 * i) = J.block<3, 3>(0, 3 * i).transpose() * (-1) * Force.segment<3>(3 * i);
    }
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

    // 몸통 des 좌표
    TH_msg.data.push_back(Body_Pos(X));
    TH_msg.data.push_back(Body_Pos(Y));
    TH_msg.data.push_back(Body_Pos(Z));

    Eigen::Vector3d Err_Pos_ = CENT.Get_Error_Pose();
    Eigen::Vector3d Err_Ori_ = CENT.Get_Error_R();

    // SRBM 추출 힘
    TH_msg.data.push_back(Err_Pos_(X));
    TH_msg.data.push_back(Err_Pos_(Y));
    TH_msg.data.push_back(Err_Pos_(Z));

    TH_msg.data.push_back(Err_Ori_(X));
    TH_msg.data.push_back(Err_Ori_(Y));
    TH_msg.data.push_back(Err_Ori_(Z));

    count += 1;

    TH_msg.data.push_back(count);

    pub_TH_.publish(TH_msg);

    // PLOT---------------------------------------------------------------------------------------------------------------------------------------------------------

    // LOGDATA------------------------------------------------------------------------------------------------------------------------------------------------------

    // LOGDATA------------------------------------------------------------------------------------------------------------------------------------------------------
}