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

    pub_leg_cmd_ = nh_.advertise<std_msgs::Float64MultiArray>(go2_topic_leg_command_, queue_size);

    controlmode = INIT;
    Recieved_Joint_State = false;

    q_.setZero(12);
    dq_.setZero(12);
    torque_.setZero(12);

    // homing | 0, 0.67, -1.40, 0, 0.67, -1.40, 0, 0.67, -1.40, 0, 0.67, -1.40;

}

void go2_controller::Command(bool flag)
{
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
            // Desired한 행렬 만들어내기
            Eigen::VectorXd q_desired(12);
            q_desired << 0, 0.67, -1.40, 0, 0.67, -1.40, 0, 0.67, -1.40, 0, 0.67, -1.40;
            
            Eigen::VectorXd dq_desired = Eigen::VectorXd::Zero(12); // 이거 의미가 있나?

            // PD 제어 Gain
            double K_p = 100.0;
            double K_d = 1.0;

            torque_ = K_p * (q_desired - q_) - K_d * dq_;

            break;
        }
        }

        SendCommandsToRobot();
    }
}



void go2_controller::Run()
{
    ROS_INFO("Running the torque control loop .................");

    const ros::Duration control_period_(1.0 / 500.0); //500hz

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
            msg.data.push_back(torque_(i));
        }
    }
    pub_leg_cmd_.publish(msg);

    msg.data.clear();
}
