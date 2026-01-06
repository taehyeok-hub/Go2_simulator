#include "plate_bot_controller.h"

void MainThreadRun(plate_bot_controller &controller)
{
    controller.Init();
    controller.Run();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "plate_bot_node");
    ros::NodeHandle nh;

    // Variable
    const double freq = 1000;

    std::string plate_bot_topic_leg_state;
    std::string plate_bot_topic_leg_command;

    // Topic names
    if (!nh.getParam("plate_bot_topic_leg_state", plate_bot_topic_leg_state))
    {
        ROS_ERROR("Couldn't retrieve the topic name for the state of the leg.");
        return -1;
    }
    if (!nh.getParam("plate_bot_topic_leg_command", plate_bot_topic_leg_command))
    {
        ROS_ERROR("Couldn't retrieve the topic name for commanding the  leg.");
        return -1;
    }

    plate_bot_controller plate_bot_controller(nh, plate_bot_topic_leg_state, plate_bot_topic_leg_command, freq);

    std::thread main_thread(MainThreadRun, std::ref(plate_bot_controller));

    main_thread.join();

    return 0;

}