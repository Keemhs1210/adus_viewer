#include "viewer.hpp"

void SigintHandler(int sig)
{
    ROS_INFO("Shutting down ROS node...");
    ros::shutdown(); // ROS 종료
}

int32_t main(int32_t argc, char **argv)
 {
    ros::init(argc, argv, "adus_viewer_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    signal(SIGINT, SigintHandler);
    std::shared_ptr<AdusViewer> adus_viewer = std::make_shared<AdusViewer>(nh, private_nh);
    adus_viewer->AdusInit();
    ROS_INFO("ROS Shutdown");
    ros::shutdown();
    return 0;
}