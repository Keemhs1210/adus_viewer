#include "path_read.h"

path_read::path_read(/* args */)
{
    // Initiate ROS node
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");	

	private_nh.param("frame_id", frame_id_, std::string("map"));

	private_nh.param("path_01", path_01_dir_, std::string("/home/kkyu/hil_ws/path/test_path_1000.txt"));
	private_nh.param("path_02", path_02_dir_, std::string("/home/kkyu/hil_ws/path/test_path_kk.txt"));
	private_nh.param("path_03", path_03_dir_, std::string("/home/kkyu/hil_ws/path/test_path_yy.txt"));

	pub_vis_path_ = nh.advertise<nav_msgs::Path>("vis_path", 10, true);
	pub_choice_path_ = nh.advertise<adss_msgs::DBZ03_Route>("choice_path", 10, true);
	pub_first_pt_ = nh.advertise<visualization_msgs::Marker>("first_pt", 1, true);
	pub_last_pt_ = nh.advertise<visualization_msgs::Marker>("last_pt", 1, true);

	sub_path_sel_msg_ = nh.subscribe("path_sel_msg", 10, &path_read::path_sel_callback, this);
}

path_read::~path_read()
{
}

void path_read::path_sel_callback(const adss_msgs::ZZZ_000_path_sel_msg &msg)
{
	double first_pt_x, first_pt_y, last_pt_x, last_pt_y = 0;
	
	// compare cplt_state
	if(prv_path_sel_cplt_ != msg.path_sel_cplt){

		if(msg.path_num == 1){
			choice_path_ = path_01_;

			init_x_ = choice_path_[0].lat;
			init_y_ = choice_path_[0].lng;
			
			first_pt_y = choice_path_[0].lat - init_x_;
			first_pt_x = choice_path_[0].lng - init_y_;
			
			last_pt_y = choice_path_[choice_path_.size()-1].lat - init_x_;
   			last_pt_x = choice_path_[choice_path_.size()-1].lng - init_y_;
		}
		else if(msg.path_num == 2){
			choice_path_ = path_02_;

			init_x_ = choice_path_[0].lat;
			init_y_ = choice_path_[0].lng;

			first_pt_y = choice_path_[0].lat - init_x_;
			first_pt_x = choice_path_[0].lng - init_y_;
			
			last_pt_y = choice_path_[choice_path_.size()-1].lat - init_x_;
   			last_pt_x = choice_path_[choice_path_.size()-1].lng - init_y_;
		}
		else if(msg.path_num == 3){
			choice_path_ = path_03_;

			init_x_ = choice_path_[0].lat;
			init_y_ = choice_path_[0].lng;

			first_pt_y = choice_path_[0].lat - init_x_;
			first_pt_x = choice_path_[0].lng - init_y_;
			
			last_pt_y = choice_path_[choice_path_.size()-1].lat - init_x_;
   			last_pt_x = choice_path_[choice_path_.size()-1].lng - init_y_;
		}

		final_sel_path_pub(choice_path_);

		visualization_msgs::Marker rviz_first_pt;   
		rviz_point_draw(rviz_first_pt, first_pt_x, first_pt_y, 0.1, 0.1, 0.1, 1, 0, 0);
		pub_first_pt_.publish(rviz_first_pt);

		visualization_msgs::Marker rviz_last_pt;   
		rviz_point_draw(rviz_last_pt, last_pt_x, last_pt_y, 0.1, 0.1, 0.1, 1, 1, 0);
		pub_last_pt_.publish(rviz_last_pt);
	}

	

	prv_path_sel_cplt_ = msg.path_sel_cplt;
}

void path_read::read_rddf_path(std::string file_directory, std::vector<waypoints> &origin_path)
{
    std::ifstream rddf(file_directory);                                         
    std::string way_point;
    std::string point_0;                                                     
    float_t x_0, y_0;
	float_t vel;
	float_t mission;

    getline(rddf, point_0, '\n');
    point_0 = std::regex_replace(point_0, std::regex(","), "\t");
    std::istringstream zero_point(point_0);
    zero_point >> x_0 >> y_0 >> vel >> mission;

	waypoints wp;  
	
	wp.lat = (float_t)x_0;
	wp.lng = (float_t)y_0;
	wp.vel = (int16_t)vel;
	wp.mission = (uint8_t)mission;  

	// std::cout << x_0 << std::endl;

                                                                 

    // origin path vector
    while(getline(rddf, way_point))                                             
    {
        way_point = std::regex_replace(way_point, std::regex(","), "\t");       
        std::istringstream point_data(way_point);                                                                   
        double index_double_type;                                              

        origin_path.push_back(wp);                                       

        point_data >> x_0 >> y_0 >> vel >> mission;  

		wp.lat = (float_t)x_0;
		wp.lng = (float_t)y_0;
		wp.vel = (int16_t)vel;
		wp.mission = (uint8_t)mission;         
    }
    origin_path.push_back(wp);
}

nav_msgs::Path path_read::wp_2_nav_path(std::vector<waypoints> wp_path)
{
	nav_msgs::Path g_path;
	g_path.poses.clear();

	waypoints pt_f;

	g_path.header.frame_id = frame_id_; 

	pt_f.lat = wp_path[0].lat;
	pt_f.lng = wp_path[0].lng;

    for(int i = 0; i < wp_path.size(); i++){
        geometry_msgs::PoseStamped pt;

		// display factor
        pt.pose.position.y = (wp_path[i].lat - init_x_) * vis_scale_;//(wp_path[i].lat - 37) * 1000;
        pt.pose.position.x = (wp_path[i].lng - init_y_) * vis_scale_;//(wp_path[i].lng - 126) * 1000;
        pt.pose.position.z = 0;

        g_path.poses.push_back(pt);
    }

    return g_path;
}

void path_read::final_sel_path_pub(std::vector<waypoints> path)
{
	adss_msgs::DBZ03_Route final_sel_path;
	adss_msgs::DEZ28_WayPoint tmp_path;

	uint8_t mission_tmp = 0;
	int16_t velocity_tmp = 0;

	for(int i = 0; i < path.size(); i++){
		tmp_path.d_Lat_10000x1.push_back(path[i].lat);
		tmp_path.d_Long_10000x1.push_back(path[i].lng);
		tmp_path.i16_Velocity_10000x1.push_back(velocity_tmp);		
		tmp_path.u8_Mission_10000x1.push_back(mission_tmp);
    }

	final_sel_path.st_e_DEZ28_WayPoint_1x1 = tmp_path;

	pub_choice_path_.publish(final_sel_path);
}


void path_read::rviz_point_draw(visualization_msgs::Marker &pt, double pt_x, double pt_y, float x, float y, float z, float r, float g, float b)
{
	pt.points.clear();
    geometry_msgs::Point point;

    pt.header.frame_id = frame_id_;   
    pt.header.stamp = ros::Time::now();
    pt.ns = "points";
    pt.id = 0;
    pt.type = visualization_msgs::Marker::POINTS;
    pt.action = visualization_msgs::Marker::ADD;
    pt.scale.x = x;
    pt.scale.y = y;
    pt.scale.z = z;
    pt.color.r = r;
    pt.color.g = g;
    pt.color.b = b;
    pt.color.a = 1.0;

    point.x = pt_x * vis_scale_; //(pt[pt.size()-1].lat - 37) * 1000;
    point.y = pt_y * vis_scale_; //(pt[pt.size()-1].lng - 126) * 1000;
    point.z = 0;
    
    pt.points.push_back(point);
}

void path_read::rddf_read()
{
	ros::Rate loop_rate(10);

	read_rddf_path(path_01_dir_, path_01_);
	read_rddf_path(path_02_dir_, path_02_);
	read_rddf_path(path_03_dir_, path_03_);

	while(ros::ok())
	{
		ros::spinOnce();

		pub_vis_path_.publish(wp_2_nav_path(choice_path_));

		loop_rate.sleep();
	}
}