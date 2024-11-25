#include <ros/ros.h>
#include <ros/ros.h>

#include "key_input.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "key_input_node");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    key_input input_key;
    input_key.key_in();

    return 0;
}