#include "path_maker_kkyu.h"

//offset_pt chk!!!
//heading tune!!!
path_maker_kkyu::path_maker_kkyu(/* args */)
{
    // Initiate ROS node
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    //include function
    inter_func_ = new interpolation_kkyu();
    func_ = new func_kkyu();
    utm_transfer_ = new utm_transfer();

    //param
    private_nh.param("lla_traj", trj_, std::string("/home/kkyu/hil_ws/path/origin_trajectory_lla/"));
    private_nh.param("utm_path", path_, std::string("/home/kkyu/hil_ws/path/rddf_path_utm/"));
    private_nh.param("lla_route", route_, std::string("/home/kkyu/hil_ws/path/rddf_path_lla/"));

    private_nh.param("frame_id", frame_id_, std::string("map"));
    private_nh.param("interpolation_num", interpolation_num_, 100);
    private_nh.param("locate_dist", locate_dist_, 1.0);



    //pub ... sub
    pub_cur_pt_ = nh.advertise<visualization_msgs::Marker>("cur_pt", 1, true);
    pub_move_trajectory_ = nh.advertise<visualization_msgs::Marker>("move_tr", 1, true);
    sub_localizer_ = nh.subscribe("gps", 10, &path_maker_kkyu::localizerCallback, this);        

    //save_file_name ... time
    std::time_t get_time;
    std::tm *now;
    get_time = std::time(nullptr);
    now = std::localtime(&get_time);

    std::stringstream save_time;
    save_time << std::setw(2) << std::setfill('0') << now->tm_year + 1900 - 2000 << std::setw(2) << std::setfill('0') << now->tm_mon + 1
     << std::setw(2) << std::setfill('0') << now->tm_mday << std::setw(2) << std::setfill('0') << now->tm_hour
     << std::setw(2) << std::setfill('0') << now->tm_min << std::setw(2) << std::setfill('0') << now->tm_sec << std::endl;

    save_name_ = save_time.str();
}

path_maker_kkyu::~path_maker_kkyu()
{
    if(inter_func_){
        delete inter_func_;
    }
    if(func_){
        delete func_;
    }
    if(utm_transfer_){
        delete utm_transfer_;
    }

}

// void path_maker_kkyu::localizerCallback(const geometry_msgs::PoseStamped::ConstPtr &localizer_pose)
void path_maker_kkyu::localizerCallback(const morai_msgs::GPSMessage::ConstPtr &localizer_pose)
{
    double roll, pitch, yaw;
    double cur_x, cur_y, cur_z;

    lla_pt_.cur_x = localizer_pose->latitude;
    lla_pt_.cur_y = localizer_pose->longitude;
    lla_pt_.cur_z = 0;

    utm_transfer_->LatLonToUTMXY(lla_pt_.cur_x, lla_pt_.cur_y, 52, utm_pt_.cur_x, utm_pt_.cur_y);

    if(prv_ == 0){
        utm_offset_pt_.x_point = utm_pt_.cur_x;
        utm_offset_pt_.y_point = utm_pt_.cur_y;
        prv_ = utm_pt_.cur_x;
    }

    vis_shift_pt_.cur_x = utm_pt_.cur_x - utm_offset_pt_.x_point;
    vis_shift_pt_.cur_y = utm_pt_.cur_y - utm_offset_pt_.y_point;
    vis_shift_pt_.cur_z = utm_pt_.cur_z - utm_offset_pt_.z_point;
}

void path_maker_kkyu::relocate_calc(double locate_dist, int n0, std::vector<waypoints> interpol_path, double &calc_dist, int &id_n)
{
    int ni = 0;
    int i = 0;

    double x, y, k;

    bool b_tmp0 = true;
    bool b_tmp1 = true;
    bool s0 = true;

    double loc_dist = (locate_dist <= 0) ? eps1 : locate_dist;
    loc_dist = loc_dist * 0.99;
    int path_size = interpol_path.size();

    while(s0)
    {
        ni = n0 + i;
        x = interpol_path[n0].x_point - interpol_path[ni].x_point;
        y = interpol_path[n0].y_point - interpol_path[ni].y_point;

        k = sqrt(pow(x, 2) + pow(y, 2));

        b_tmp0 = (k >= loc_dist) ? true : false;
        b_tmp1 = ((path_size - 1) == ni) ? true : false;

        s0 = !(b_tmp0 || b_tmp1);

        i++;
    }
    calc_dist = k;
    id_n = ni;
}

void path_maker_kkyu::re_locate(double locate_dist, std::vector<waypoints> interpol_path, std::vector<waypoints> &relocate_path)
{
    double calc_dist;
    int id_n;
    int n0 = 0;
    int i = 0;

    bool s0 = true;
    int path_size = interpol_path.size();

    waypoints wp;

    wp.x_point = interpol_path[0].x_point;
    wp.y_point = interpol_path[0].y_point;
    wp.index = 0;
    relocate_path.push_back(wp);

    while(s0)
    {
        i++;
        relocate_calc(locate_dist, n0, interpol_path, calc_dist, id_n);
        s0 = ((path_size - 1) == id_n) ? false : true;
        n0 = id_n;

        wp.index = i;
        wp.x_point = interpol_path[n0].x_point;
        wp.y_point = interpol_path[n0].y_point;
        
        relocate_path.push_back(wp);
    }
}

void path_maker_kkyu::rviz_cur_pose_draw(visualization_msgs::Marker &cur_point, double x, double y, double z, double r, double g, double b, double x_pt, double y_pt)
{
    geometry_msgs::Point point;

    cur_point.header.frame_id = frame_id_;   
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

    point.x = x_pt;
    point.y = y_pt;
    point.z = 0;
    
    cur_point.points.push_back(point);
}

void path_maker_kkyu::path_making()
{
    ros::Rate loop_rate(10);

    std::ostringstream ss_0;
    ss_0 << trj_ << save_name_ << "_trj.txt" << std::endl;
    std::string str_0 = ss_0.str();
    str_0.erase(remove(str_0.begin(), str_0.end(), '\n'), str_0.end());
    std::ofstream ofs_origin(str_0);

    std::ostringstream ss_1;
    ss_1 << path_ << save_name_ << "_rddf.txt" << std::endl;
    std::string str_1 = ss_1.str();
    str_1.erase(remove(str_1.begin(), str_1.end(), '\n'), str_1.end());
    std::ofstream ofs(str_1);


    std::ostringstream ss_2;
    ss_2 << route_ << save_name_ << "_route.txt" << std::endl;
    std::string str_2 = ss_2.str();
    str_2.erase(remove(str_2.begin(), str_2.end(), '\n'), str_2.end());
    std::ofstream ofs_route(str_2);

    waypoints lla_pt;
    std::vector<waypoints> lla_trajectory;

    waypoints utm_pt;
    std::vector<waypoints> utm_trajectory;

    int cnt = 0;
    
    while(ros::ok())
    {
        ros::spinOnce();
       
        if(cnt < 10){
            std::cout << "==========.....Waiting.....==========" << std::endl;
            cnt++;
        }
        else{
            cnt = 10;

            std::cout.precision(16);
            lla_pt.x_point = lla_pt_.cur_x;
            lla_pt.y_point = lla_pt_.cur_y;
            lla_pt.z_point = lla_pt_.cur_z;
            lla_trajectory.push_back(lla_pt);

            std::cout.precision(16);
            utm_pt.x_point = utm_pt_.cur_x;
            utm_pt.y_point = utm_pt_.cur_y;
            utm_pt.z_point = utm_pt_.cur_z;
            utm_trajectory.push_back(utm_pt);
            
            std::cout << "cur_x :  " << lla_pt_.cur_x << "    " << "cur_y :  " << lla_pt_.cur_y << "    " << "cur_yaw :  " << lla_pt_.cur_yaw << std::endl;

            visualization_msgs::Marker rviz_vehicle_cur_pos_;   
            rviz_cur_pose_draw(rviz_vehicle_cur_pos_, 3, 3, 3, 0, 1, 0, vis_shift_pt_.cur_x, vis_shift_pt_.cur_y);
            pub_cur_pt_.publish(rviz_vehicle_cur_pos_);

            rviz_cur_pose_draw(rviz_trajectory_, 1, 1, 1, 1, 1, 0, vis_shift_pt_.cur_x, vis_shift_pt_.cur_y);
            pub_move_trajectory_.publish(rviz_trajectory_);
        }

        loop_rate.sleep();
    }

    // ===origin_trajectory===
    std::cout.precision(16);
    std::cout << "==========.....Path Making.....==========" << std::endl;

    for(int i = 0; i < lla_trajectory.size(); i++)
    {
        ofs_origin.precision(16);
        ofs_origin << lla_trajectory[i].x_point << "\t" << lla_trajectory[i].y_point << '\t' << lla_trajectory[i].z_point << std::endl;
    }

    // ===interpolation===
    std::vector<double> x_vec;
    std::vector<double> y_vec;

    for(int i = 0; i < utm_trajectory.size(); i+=2)
    {
        x_vec.push_back(utm_trajectory[i].x_point);
        y_vec.push_back(utm_trajectory[i].y_point);
    }

    std::vector<double> x_interpol_vec;
    std::vector<double> y_interpol_vec;

    inter_func_->hermite_interpolation(x_vec, interpolation_num_, x_interpol_vec);
    inter_func_->hermite_interpolation(y_vec, interpolation_num_, y_interpol_vec);

    std::vector<waypoints> interpolation_path;

    for(int j = 0; j < x_interpol_vec.size(); j++)
    {
        waypoints pt0;

        pt0.x_point = x_interpol_vec[j];
        pt0.y_point = y_interpol_vec[j];

        interpolation_path.push_back(pt0);
    }

    // ===re locate===
    std::vector<waypoints> relocate_path;
    re_locate(locate_dist_, interpolation_path, relocate_path);


    int vel = 0;
    int mission = 0;

    for(int i = 0; i < relocate_path.size(); i++)
    {
        ofs.precision(16);
        ofs << relocate_path[i].x_point << "\t" << relocate_path[i].y_point << "\t" << vel << "\t" << mission << std::endl;
    }

    double lat, route_lat;
    double lng, route_lng;

    for(int i = 0; i < relocate_path.size(); i++)
    {
        utm_transfer_->UTMXYToLatLon (relocate_path[i].x_point, relocate_path[i].y_point, zone_, 0, lat, lng);
        route_lat = utm_transfer_->RadToDeg(lat);
        route_lng = utm_transfer_->RadToDeg(lng);

        ofs_route.precision(16);
        ofs_route << route_lat << "\t" << route_lng << "\t" << vel << "\t" << mission << std::endl;
    }

    std::cout << "!!!!!!!!!!!!!!!  Complete  !!!!!!!!!!!!!!!" << std::endl;
}
