#ifndef __FUNC_KKYU_H__
#define __FUNC_KKYU_H__

#include "func_kkyu.h"

#include "def.h"

class func_kkyu
{
private:
    /* data */

public:
    func_kkyu(/* args */);
    ~func_kkyu();

public:
    double uc_dist_calc (double x_0, double y_0, double x_1, double y_1);
    double yaw_deg_transfer (double yaw);
    int near_point_find (std::vector<waypoints> path, double x, double y);
    int similar_near_point_find (std::vector<waypoints> path, double x, double y);

    void lat_move_point (double dist, double yaw, double x, double y, double &lat_x, double &lat_y);
    void lng_move_point (double dist, double yaw, double x, double y, double &lng_x, double &lng_y);

    double error_dist_calc (std::vector<waypoints> path, double x_point, double y_point);
    double two_heading_find(double x_0, double y_0, double x_1, double y_1);
};

#endif




