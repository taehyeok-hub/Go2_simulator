#include "go2_controller.h"

void MainThreadRun(go2_controller &controller)
{
    controller.Init();
    controller.Run();
}

void CentThreadRun(go2_controller &controller)
{
    controller.CentRun();
}

void PlotThreadRun(go2_controller &controller)
{
    controller.PlotRun();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "go2_node"); // ros에 go2_node라고 연결함.
    ros::NodeHandle nh;

    const double freq = FREQUENCY;

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

    go2_controller go2_controller(nh, go2_topic_leg_state, go2_topic_leg_command, freq); // 객체 생성

    // go2_controller.Init(); // 초기화 진행

    std::thread main_thread(MainThreadRun, std::ref(go2_controller));
    std::thread plot_thread(PlotThreadRun, std::ref(go2_controller));
    std::thread cent_thread(CentThreadRun, std::ref(go2_controller));

    main_thread.join();
    plot_thread.join();
    cent_thread.join();

    return 0;
}