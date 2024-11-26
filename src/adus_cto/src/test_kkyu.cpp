#include <ros/ros.h>
#include <numeric>
#include <std_msgs/String.h>
#include <nmea_msgs/Sentence.h>
#include <std_msgs/UInt8.h>
#include <iostream>
#include <mutex>
#include <def.h>

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

void lng_move_point (double dist, double yaw, double x, double y, double &lng_x, double &lng_y)
{
    lng_x = ( sin( (yaw) * M_PI / 180 ) * (-dist) ) + x;
    lng_y = ( cos( (yaw) * M_PI / 180 ) * (dist) ) + y; 
}

double front_path_curvature(waypoints front_pt, cur_pose cur_pt, double dist)
{
    double q, w, e, curvature_k;
    
    q = atan2((front_pt.y_point - cur_pt.cur_y), (front_pt.x_point - cur_pt.cur_x));
    w = ((M_PI / 2) - q) - (cur_pt.cur_yaw * M_PI /180);
    e = 2 * sin(w);
    
    return curvature_k = (e / dist); // * 180 / M_PI;
}


void cir_o_pt_find(double curvature_k, cur_pose cur_pt, waypoints ref_pt, waypoints &cir_o_pt, bool &l_r_bool)
{
    curvature_k *= -1;
    double fdx, fdy;
    fdx = cur_pt.cur_x - ref_pt.x_point;
    fdy = cur_pt.cur_y - ref_pt.y_point;

    double fxc, fyc;
    fxc = (cur_pt.cur_x + ref_pt.x_point) / 2;
    fyc = (cur_pt.cur_y + ref_pt.y_point) / 2;

    double fd0, fd1;
    fd0 = pow((1 / curvature_k), 2);
    fd1 = pow(((sqrt(pow(fdx, 2) + pow(fdy, 2))) / 2), 2);

    double f_offset_tmp = ((fd0 - fd1) < 0) ? (fd1 - fd0) : (fd0 - fd1);
    double f_offset = sqrt(f_offset_tmp);

    double fpx, fpy;
    fpx = f_offset * (fdy / (sqrt(pow(fdx, 2) + pow(fdy, 2))));
    fpy = f_offset * (fdx / (sqrt(pow(fdx, 2) + pow(fdy, 2))));

    if(curvature_k >= 0){
        l_r_bool = true;
        cir_o_pt.x_point = fxc - fpx;
        cir_o_pt.y_point = fyc + fpy;
    }
    else{
        l_r_bool = false;
        cir_o_pt.x_point = fxc + fpx;
        cir_o_pt.y_point = fyc - fpy;
    }
}

void cir_make(cur_pose cur_pt, waypoints ref_pt, waypoints cir_o_pt, double curvature_k, bool l_r_bool, std::vector<waypoints> &cir_vec)
{
    curvature_k *= -1;
    double r = 1 / curvature_k;
    
    double c_deg0 = (l_r_bool == true) ? 180 : 0;
    double c_deg1 = ((cur_pt.cur_x - cir_o_pt.x_point) > 0) ? 180 : 0;
    double c_deg2 = (atan((cur_pt.cur_y - cir_o_pt.y_point) / (cur_pt.cur_x - cir_o_pt.x_point))) / M_PI * 180;
    double c_deg3 = c_deg0 + c_deg1 + c_deg2 + curvature_k;

    double l_deg0 = (l_r_bool == true) ? 180 : 0;
    double l_deg1 = ((ref_pt.x_point - cir_o_pt.x_point) > 0) ? 180 : 0;
    double l_deg2 = (atan((ref_pt.y_point - cir_o_pt.y_point) / (ref_pt.x_point - cir_o_pt.x_point))) / M_PI * 180;
    double l_deg3 = l_deg0 + l_deg1 + l_deg2 + curvature_k;

    double deg_tmp = l_deg3 - c_deg3;
    double deg_f = (180 > abs(deg_tmp)) ? abs(deg_tmp) : abs(deg_tmp) - 360;

    double cir_ang = (deg_tmp < 0) ? (deg_f * -1) : deg_f;

    int sample_num = 50;
    double scale = cir_ang / (double)sample_num;  
    for(int i = 0; i < sample_num + 1; i++){
        waypoints pt;

        double c = (scale * (double)i + c_deg3) * M_PI / 180;

        pt.x_point = cir_o_pt.x_point + (r * cos(c));
        pt.y_point = cir_o_pt.y_point + (r * sin(c));

        cir_vec.push_back(pt);
    }
}

// way point -> nav path func
nav_msgs::Path wp_2_nav_path(std::vector<waypoints> wp_path)
{
	nav_msgs::Path g_path;
    g_path.header.frame_id = "map";
    // g_path.header.frame_id = frame_id_;   

    for(int i = 0; i < wp_path.size(); i++){
        geometry_msgs::PoseStamped pt;

        pt.pose.position.x = wp_path[i].x_point;
        pt.pose.position.y = wp_path[i].y_point;
        pt.pose.position.z = wp_path[i].z_point;

        g_path.poses.push_back(pt);
    }

    return g_path;
}



void rviz_cur_pose_draw(visualization_msgs::Marker &cur_point, float x, float y, float z, float r, float g, float b, double cur_x, double cur_y)
{
    geometry_msgs::Point point;

    cur_point.header.frame_id = "map";   
    cur_point.header.stamp = ros::Time::now();
    cur_point.ns = "points";
    cur_point.id = 0;
    cur_point.type = visualization_msgs::Marker::POINTS;
    cur_point.action = visualization_msgs::Marker::ADD;
    cur_point.scale.x = x;
    cur_point.scale.y = y;
    cur_point.scale.z = z;
    cur_point.color.r = r;
    cur_point.color.g = g;
    cur_point.color.b = b;
    cur_point.color.a = 1.0;

    point.x = cur_x;
    point.y = cur_y;
    point.z = 0.1;
    
    cur_point.points.push_back(point);
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Rate loop_rate(50);
    

    ros::Publisher pub_path_nav_curvature_;
    pub_path_nav_curvature_ = nh.advertise<nav_msgs::Path>("path_curvature_nav", 1, true);

    ros::Publisher pub_cur_pt_;
    pub_cur_pt_ = nh.advertise<visualization_msgs::Marker>("cur_pt", 1, true);

    ros::Publisher pub_ref_pt_;
    pub_ref_pt_ = nh.advertise<visualization_msgs::Marker>("ref_pt", 1, true);
    

    cur_pose cur_pt;
    cur_pt.cur_x = 0;
    cur_pt.cur_y = 0;

    double lng_move_x, lng_move_y;
    lng_move_point (3, -10, cur_pt.cur_x, cur_pt.cur_y, lng_move_x, lng_move_y);

    waypoints ref_pt;
    ref_pt.x_point = lng_move_x;
    ref_pt.y_point = lng_move_y;
    
    double curvature;
    curvature = front_path_curvature(ref_pt, cur_pt, 3);
    curvature *= -1; 

    waypoints cir_o_pt;
    bool lr;
    std::vector<waypoints> cir_vec;
    
    cir_o_pt_find(curvature, cur_pt, ref_pt, cir_o_pt, lr);
    cir_make(cur_pt, ref_pt, cir_o_pt, curvature, lr, cir_vec);

	while (ros::ok())
	{
        ros::spinOnce();

        pub_path_nav_curvature_.publish(wp_2_nav_path(cir_vec));

        visualization_msgs::Marker cur_pt_marker;
        rviz_cur_pose_draw(cur_pt_marker, 0.2, 0.2, 0.2, 1, 0, 0, 0, 0);   
        pub_cur_pt_.publish(cur_pt_marker);

        visualization_msgs::Marker ref_pt_marker;
        rviz_cur_pose_draw(ref_pt_marker, 0.2, 0.2, 0.2, 1, 0, 0, lng_move_x, lng_move_y);   
        pub_ref_pt_.publish(ref_pt_marker);
        
        loop_rate.sleep();

    }
}