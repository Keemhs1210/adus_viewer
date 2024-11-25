#include "func_kkyu.h"

func_kkyu::func_kkyu(/* args */)
{
}

func_kkyu::~func_kkyu()
{
}

// uc_dist calculate
double func_kkyu::uc_dist_calc (double x_0, double y_0, double x_1, double y_1)
{
    return sqrt( pow((x_1 - x_0), 2) + pow((y_1 - y_0), 2));
}

// angle deg transfer (0 ~ 360  >>> -180 ~180)
double func_kkyu::yaw_deg_transfer (double yaw)
{
    double q, w;
    q = yaw / abs(yaw);

    if(abs(abs(yaw) - 360) > 180){
        w = abs(yaw);
    }
    else{
        w = abs(yaw) - 360;
    }
    return q * w;
}

int func_kkyu::near_point_find (std::vector<waypoints> path, double x, double y)
{
    double dist = DBL_MAX;
    double cur_dist = 0;
    int cur_index = 0;

    path.pop_back();

    for (int i = 0; i <= path.size(); i++)
    {
        cur_dist = uc_dist_calc(x, y, path[i].x_point, path[i].y_point);         // rviz display error  ... localizer point data!!

        if (cur_dist < dist)
        {
            dist = cur_dist;
            cur_index = i;
        }
    }
    return cur_index;
}


// wp find for local path ... similar direction point find
int func_kkyu::similar_near_point_find (std::vector<waypoints> path, double x, double y)
{
    double q, dist;
    int index = 0;

    double tmp_dist = 100;
    double out_dist;

    int tmp_index = 0;
    int out_index;

    path.pop_back();

    for (int i = 0; i < path.size(); i++)
    {
        dist = uc_dist_calc(path[i].x_point, path[i].y_point, x, y);
        q = abs(yaw_deg_transfer((90 - (atan2((path[i+1].y_point - path[i].y_point), (path[i+1].x_point - path[i].x_point)) * 180 / M_PI)) - 90));

        if (q >= 359){
            out_index = tmp_index;
            out_dist = tmp_dist;
        }
        else{
            if(dist <= tmp_dist){
                out_index = i;
                tmp_index = out_index;
                
                out_dist = dist;
                tmp_dist = out_dist;
            }
            else{
                out_index = tmp_index;
                
                out_dist = tmp_dist;
            }
        }
    }
    return out_index;
}

// lat_point_make
void func_kkyu::lat_move_point (double dist, double yaw, double x, double y, double &lat_x, double &lat_y)
{
    lat_x = -( cos( (yaw) * M_PI / 180 ) * dist ) + x;
    lat_y = ( sin( (yaw) * M_PI / 180 ) * dist ) + y; 
}

// lng_point_make
void func_kkyu::lng_move_point (double dist, double yaw, double x, double y, double &lng_x, double &lng_y)
{
    lng_x = ( sin( (yaw) * M_PI / 180 ) * (-dist) ) + x;
    lng_y = ( cos( (yaw) * M_PI / 180 ) * (-dist) ) + y; 
}

// cur point error distance from path
double func_kkyu::error_dist_calc (std::vector<waypoints> path, double x_point, double y_point)
{
    int near_index;
    double x_0, x_1, x_2, y_0, y_1, y_2;
    double x, y;
    double q, w, e, r;
    double dir;

    path.pop_back();
    near_index = near_point_find(path, x_point, y_point);
    
    x_0 = x_point;
    y_0 = y_point;

    if(near_index == 0){
        x_1 = path[near_index].x_point;
        y_1 = path[near_index].y_point;
        x_2 = path[near_index + 1].x_point;
        y_2 = path[near_index + 1].y_point;
    }
    else if(near_index == path.size()-1){
        x_1 = path[near_index - 1].x_point;
        y_1 = path[near_index - 1].y_point;
        x_2 = path[near_index].x_point;
        y_2 = path[near_index].y_point;
    }
    else{
        x_1 = path[near_index - 1].x_point;
        y_1 = path[near_index - 1].y_point;
        x_2 = path[near_index + 1].x_point;
        y_2 = path[near_index + 1].y_point;
    }

    x = x_2 - x_1;    
    x = x == 0 ? 0.0001 : x;
    y = y_2 - y_1;

    q = abs( ((x_0 * y/x) - y_0) + (y_1 - (x_1 * y/x)) );
    w = sqrt (pow(y/x, 2) + 1 );
    e = fmod( ( (M_PI/2 - atan2 (y, x)) + 2*M_PI ), (2*M_PI) );
    r = (y/x) * (x_0 - x_1) + y_1;   

    if(e >= M_PI && e < 2*M_PI){
        if(r > y_0) dir = -1;
        else dir = 1;
    }
    else{
        if(r > y_0) dir = 1;
        else dir = -1;
    }

    return q/w*dir;   // error distance
}

double func_kkyu::two_heading_find(double x_0, double y_0, double x_1, double y_1)
{
    double dx, dy;
    double heading;

    dx = x_1 - x_0;
    dy = y_1 - y_0;

    heading = atan2(dy, dx) * 180 / M_PI; 
    heading *= -1;
    if(heading < 0){
        heading += 360;
    }
    else{
        heading = heading;
    }
    heading -= 90;

    return heading;
}
