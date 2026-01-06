#include "plate_bot_controller.h"

void plate_bot_controller::Init()
{
    // Reset Gazebo Simualtion
    system("clear");
    // std_srvs::Empty reset;
    // ros::service::call("/gazebo/reset_simulation", reset);
    std::cout << "Reset Gazebo Simulation" << std::endl;  
    nh_.ok();

    uint32_t queue_size = 10;

    // Subscribers
    sub_leg_state_ = nh_.subscribe(plate_bot_topic_leg_state_, queue_size, &plate_bot_controller::StateLegCallback, this, ros::TransportHints().reliable().tcpNoDelay());

    pub_leg_cmd_ = nh_.advertise<std_msgs::Float64MultiArray>(plate_bot_topic_leg_command_, queue_size);
    pub_real_data_ = nh_.advertise<std_msgs::Float64MultiArray>("data_real_stream", queue_size);
    pub_ref_data_ = nh_.advertise<std_msgs::Float64MultiArray>("data_ref_stream", queue_size);
    pub_data_ = nh_.advertise<std_msgs::Float64MultiArray>("data_stream", queue_size);

    Recieved_Joint_State = false;
    current_time_ = 0.0;
    Roll_time = 0.0;
    Pitch_time = 0.0;
    period_roll = 4.0;
    period_pitch = 3.0;

    q_.setZero(2);
    dq_.setZero(2);
    torque_.setZero(2);
    target_joint_pos_.setZero(2);
    currentPosition.setZero(2);
    Plate_angle.setZero(2);
    Upper_Angle.setZero(2);
    Lower_Angle.setZero(2);
}

void plate_bot_controller::Command(bool flag)
{
    Upper_Angle << 0.2, 0.2;
    Lower_Angle << -0.2, -0.2;
    if(Roll_flag == 0)
    {
        currentPosition(0)  = q_(0);
        Roll_flag = 1;
    }        

    else if(Roll_flag == 1)
    {
        Plate_angle(0) = currentPosition(0) + (Lower_Angle(0) - currentPosition(0)) * 0.5 * (1 - cos(3.14 * (Roll_time) / period_roll));

        if(Roll_time < period_roll)
        {
            Roll_time += 0.001;
        }
        else if(Roll_time >= period_roll)
        {
            Roll_flag = 2;
            Roll_time = 0;
        }
    }

    else if(Roll_flag == 2)
    {
        currentPosition(0)  = q_(0);
        Roll_flag = 3;
    }

    else if(Roll_flag == 3)
    {
        Plate_angle(0) = currentPosition(0) + (Upper_Angle(0) - currentPosition(0)) * 0.5 * (1 - cos(3.14 * (Roll_time) / period_roll));

        if(Roll_time < period_roll)
        {
            Roll_time += 0.001;
        }
        else if(Roll_time >= period_roll)
        {
            Roll_flag = 0;
            Roll_time = 0;
        }
    }


    if(Pitch_flag == 0)
    {
        currentPosition(1)  = q_(1);
        Pitch_flag = 1;
    }        

    else if(Pitch_flag == 1)
    {
        Plate_angle(1) = currentPosition(1) + (Lower_Angle(1) - currentPosition(1)) * 0.5 * (1 - cos(3.14 * (Pitch_time) / period_pitch));

        if(Pitch_time < period_pitch)
        {
            Pitch_time += 0.001;
        }
        else if(Pitch_time >= period_pitch)
        {
            Pitch_flag = 2;
            Pitch_time = 0;
        }
    }

    else if(Pitch_flag == 2)
    {
        currentPosition(1)  = q_(1);
        Pitch_flag = 3;
    }

    else if(Pitch_flag == 3)
    {
        Plate_angle(1) = currentPosition(1) + (Upper_Angle(1) - currentPosition(1)) * 0.5 * (1 - cos(3.14 * (Pitch_time) / period_pitch));

        if(Pitch_time < period_pitch)
        {
            Pitch_time += 0.001;
        }
        else if(Pitch_time >= period_pitch)
        {
            Pitch_flag = 0;
            Pitch_time = 0;
        }
    }

    torque_ = 10000 * (Plate_angle - q_) - 100 * (dq_);
    std::cout << Plate_angle(0) << "   |   " << Plate_angle(1) << "   |   " << q_(0) << "   |   " << q_(1) << std::endl;
    SendCommandsToRobot();

}

void plate_bot_controller::Run()
{
    ROS_INFO("Running the torque control loop .................");

    const ros::Duration control_period_(1.0 / 1000.0);

    ros::AsyncSpinner spinner(4);
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

void plate_bot_controller::StateLegCallback(const sensor_msgs::JointState &state)
{
    q_(0) = state.position[1];
    q_(1) = state.position[0];

    dq_(0) = state.velocity[1];
    dq_(1) = state.velocity[0];

    Recieved_Joint_State = true;
}

void plate_bot_controller::SendCommandsToRobot()
{
    std_msgs::Float64MultiArray msg;

    for (size_t i = 0; i < 2; i++)
    {
        msg.data.push_back(torque_(i));
    }
    pub_leg_cmd_.publish(msg);

    msg.data.clear();
}

void plate_bot_controller::DataStream()
{
    std_msgs::Float64MultiArray data_real_msg;
    pub_real_data_.publish(data_real_msg);

    std_msgs::Float64MultiArray data_ref_msg;
    pub_ref_data_.publish(data_ref_msg);

    std_msgs::Float64MultiArray data_msg;
    pub_data_.publish(data_msg);
}

double plate_bot_controller::Rotating_Command(double period, Eigen::VectorXd upper_radian, Eigen::VectorXd lower_radian) 
{   

}