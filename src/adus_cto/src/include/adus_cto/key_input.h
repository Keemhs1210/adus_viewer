#ifndef __KEY_INPUT_H__
#define __KEY_INPUT_H__

#include "key_input.h"
// custom message
#include <ros/ros.h>
#include <termios.h>
#include <std_msgs/UInt8.h>

#include "adss_msgs/ZZZ_000_path_sel_msg.h"

class key_input
{
public:
    key_input(/* args */);
    ~key_input();    
    
private:
    ros::Publisher pub_path_sel_msg_;

public:
    char getch();

public:
    void key_in();

};

#endif