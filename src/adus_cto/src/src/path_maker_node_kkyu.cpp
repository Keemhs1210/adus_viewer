#include <iostream>
#include <cstdlib>
#include <vector>
#include <fstream>
#include <string>

#include <ros/ros.h>

#include "path_maker_kkyu.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_maker_kkyu");
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    path_maker_kkyu maker_kkyu;
    maker_kkyu.path_making();

    return 0;
}