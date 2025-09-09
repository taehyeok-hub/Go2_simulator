#include "go2_controller.h"

void MainThreadRun(go2_controller &controller)
{
    controller.Init();
    controller.Run();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "go2_node");
    ros::NodeHandle nh;

    const double freq = 500;

    std::string go2_topic_leg_state;
    std::string go2_topic_leg_command;

    // Topic names
    if (!nh.getParam("go2_topic_leg_state", go2_topic_leg_state))
    {
        ROS_ERROR("Couldn't retrieve the topic name for the state of the leg.");
        return -1;
    }
    if (!nh.getParam("go2_topic_leg_command", go2_topic_leg_command))
    {
        ROS_ERROR("Couldn't retrieve the topic name for commanding the leg.");
        return -1;
    }

    go2_controller go2_controller(nh, go2_topic_leg_state, go2_topic_leg_command, freq);

    std::thread main_thread(MainThreadRun, std::ref(go2_controller));

    main_thread.join();

    return 0;
}