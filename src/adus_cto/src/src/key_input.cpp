#include "key_input.h"

key_input::key_input(/* args */)
{
    // Initiate ROS node
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");	

	pub_path_sel_msg_ = nh.advertise<adss_msgs::ZZZ_000_path_sel_msg> ("path_sel_msg", 1);
}

key_input::~key_input()
{
}

char key_input::getch()
{
	fd_set set;
	struct timeval timeout;
	int rv;
	char buff = 0;
	int len = 1;
	int filedesc = 0;
	FD_ZERO(&set);
	FD_SET(filedesc, &set);

	timeout.tv_sec = 0;
	timeout.tv_usec = 1000;

	rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

	struct termios old = {0};
	if (tcgetattr(filedesc, &old) < 0)
		ROS_ERROR("tcsetattr()");
	old.c_lflag &= ~ICANON;
	old.c_iflag &= ~ECHO;
	old.c_cc[VMIN] = 1;
	old.c_cc[VTIME] = 0;
	if (tcsetattr(filedesc, TCSANOW, &old) < 0)
		ROS_ERROR("tcsetattr ICANON");

	if(rv != -1 && rv != 0)
		read(filedesc, &buff, len);

	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if(tcsetattr(filedesc, TCSADRAIN, &old) < 0)
		ROS_ERROR("tcsetattr ~ICANON");

	return (buff);
}


void key_input::key_in()
{
	ros::Rate loop_rate(20);

	adss_msgs::ZZZ_000_path_sel_msg path_sel_msg;

	path_sel_msg.path_sel = 0x00;
	path_sel_msg.path_num = 0x00;
	path_sel_msg.path_sel_cplt = 0x00;
	path_sel_msg.path_reverse = 0x00;

	while(ros::ok())
	{
		ros::spinOnce();
		int c = 0;
		c = getch();

		if((path_sel_msg.path_sel == 0x00) & (c == ' ')){
			path_sel_msg.path_sel = 0x01;
		}
		else if((path_sel_msg.path_sel == 0x01) & (c == ' ')){
			path_sel_msg.path_sel = 0x00;
		}

		if(c == '1'){
			path_sel_msg.path_num = 0x01;
		}
		else if(c == '2'){
			path_sel_msg.path_num = 0x02;
		}
		else if(c == '3'){
			path_sel_msg.path_num = 0x03;
		}


		if(path_sel_msg.path_reverse == 0x00){
			if(c == 'k'){
				path_sel_msg.path_reverse = 0x01;
			}
		}
		else if(path_sel_msg.path_reverse == 0x01){
			if(c == 'k'){
				path_sel_msg.path_reverse = 0x00;
			}
		}

		if(path_sel_msg.path_sel_cplt == 0x00){
			if(c == '\n'){
				path_sel_msg.path_sel_cplt = 0x01;
			}
		}
		else if(path_sel_msg.path_sel_cplt == 0x01){
			if(c == '\n'){
				path_sel_msg.path_sel_cplt = 0x00;
			}
		}

		pub_path_sel_msg_.publish(path_sel_msg);

		loop_rate.sleep();
	}
}
