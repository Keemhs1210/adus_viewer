#ifndef __PATH_MAKER_KKYU_H__
#define __PATH_MAKER_KKYU_H__

#include "path_maker_kkyu.h"

#include "adss_msgs/ZZZ_000_path_sel_msg.h"
#include "adss_msgs/DEZ28_WayPoint.h"
#include "adss_msgs/DBZ03_Route.h"
#include "morai_msgs/GPSMessage.h"

#include <fstream>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <ctime>

#include "interpolation_kkyu.h"
#include "func_kkyu.h"
#include "def.h"
#include "utm_transfer.h"

class path_maker_kkyu
{
public:
    path_maker_kkyu(/* args */);
    ~path_maker_kkyu();

private:
    waypoints utm_offset_pt_; 
    
    cur_pose lla_pt_, utm_pt_, vis_shift_pt_;
    cur_pose lla_traj_save_, lla_rddf_save_, utm_rddf_save_;

    double prv_ = 0;
    int zone_ = 52;

    std::string trj_;
    std::string path_;
    std::string route_;
    std::string frame_id_;
    int interpolation_num_;
    double locate_dist_;

    std::string save_name_;

    ros::Publisher pub_cur_pt_;
    ros::Publisher pub_move_trajectory_;

    ros::Subscriber sub_localizer_;

    visualization_msgs::Marker rviz_trajectory_; 

    interpolation_kkyu *inter_func_;    
    func_kkyu *func_;
    utm_transfer *utm_transfer_;


public:
    void initposeCallback(const geometry_msgs::PoseStamped::ConstPtr &shift_pose);
    // void localizerCallback(const geometry_msgs::PoseStamped::ConstPtr &localizer_pose);
    void localizerCallback(const morai_msgs::GPSMessage::ConstPtr &localizer_pose);
    void relocate_calc(double locate_dist, int n0, std::vector<waypoints> interpol_path, double &calc_dist, int &id_n);
    void re_locate(double locate_dist, std::vector<waypoints> interpol_path, std::vector<waypoints> &relocate_path);
    void rviz_cur_pose_draw(visualization_msgs::Marker &cur_point, double x, double y, double z, double r, double g, double b, double x_pt, double y_pt);

public:
    void path_making();
};

#endif