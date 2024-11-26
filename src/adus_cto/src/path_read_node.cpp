#include <ros/ros.h>

#include "path_read.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_read_node");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    path_read read_path;
    read_path.rddf_read();

    return 0;
}