#ifndef __PATH_READ_H__
#define __PATH_READ_H__

#include "path_read.h"

// custom message
#include "adss_msgs/ZZZ_000_path_sel_msg.h"
#include "adss_msgs/DEZ28_WayPoint.h"
#include "adss_msgs/DBZ03_Route.h"


#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <iterator>
#include <regex>
#include <string>
#include <visualization_msgs/Marker.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <algorithm>

typedef struct waypoints
{
    double lat;
    double lng;
    int16_t vel;
    uint8_t mission;
} waypoints;

class path_read
{
public:
    path_read(/* args */);
    ~path_read();    
    
private:
    uint8_t prv_path_num_;
    uint8_t prv_path_reverse_;
    uint8_t prv_path_sel_cplt_;

    std::string frame_id_;

    std::string path_01_dir_;
    std::string path_02_dir_;
    std::string path_03_dir_;

    double init_x_, init_y_;
    double vis_scale_ = 1000;

    std::vector<waypoints> path_01_;
	std::vector<waypoints> path_02_;
	std::vector<waypoints> path_03_;

    std::vector<waypoints> choice_path_;

    ros::Publisher pub_vis_path_;
    ros::Publisher pub_choice_path_;
    ros::Publisher pub_first_pt_;
    ros::Publisher pub_last_pt_;

    ros::Subscriber sub_path_sel_msg_;
   
public:
    void path_sel_callback(const adss_msgs::ZZZ_000_path_sel_msg &msg);
    void read_rddf_path(std::string file_directory, std::vector<waypoints> &origin_path);
    nav_msgs::Path wp_2_nav_path(std::vector<waypoints> wp_path);
    void final_sel_path_pub(std::vector<waypoints> path);
    void rviz_point_draw(visualization_msgs::Marker &pt, double pt_x, double pt_y, double x, double y, double z, double r, double g, double b);
public:
    void rddf_read();
};

#endif