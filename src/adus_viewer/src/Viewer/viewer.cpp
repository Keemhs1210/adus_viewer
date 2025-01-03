#include "viewer.hpp"
#include "Input/input_topic.hpp"

using namespace std;

void AdusViewer::AdusInit()
{   
    // Subscribe callback class
    shared_ptr<InputTopic> input_topic = make_shared<InputTopic>(node_, private_nh_,
                                          &stVehicleInfo, &stEgoInfo,
                                          &stGpsInfo, &stPathInfo, &stLTrajInfo, &stTargetInfo,
                                          &stImuInfo);
    sub_image = node_.subscribe("/image_jpeg/compressed", 1, &AdusViewer::Image_callback, this);
    // Publish callback variable
    pub_Vehicle = node_.advertise<visualization_msgs::MarkerArray>("vehicle_bounding_boxes", 1);
    pub_EgoVehicle = node_.advertise<visualization_msgs::Marker>("ego_bounding_boxes", 1);
    pub_EgoHeading = node_.advertise<visualization_msgs::Marker>("ego_heading", 1);
    pub_LocalPath = node_.advertise<visualization_msgs::Marker>("local_traj", 1);
    pub_GlobalPath = node_.advertise<visualization_msgs::Marker>("global_traj", 1);
    pub_Text = node_.advertise<jsk_rviz_plugins::OverlayText>("adus_info", 1);
    pub_Image = node_.advertise<sensor_msgs::Image>("camera/image", 1);
    //main loop 10Hz
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        Visualization();
        loop_rate.sleep();
    }
    ros::shutdown();
}

void AdusViewer::Visualization()
{
    // 1. Ego & Object Marker
    DrawVehicle();
    // 2. Global Trajectory Marker
    DrawPath();
    // 3. Draw Text
    DrawText();

}

void AdusViewer::DrawVehicle()
{
    //Ego Info
    int32_t iIdx = 0;
    int32_t iFlag = 0;
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray marker_array;

    marker = DrawMarker(stEgoInfo.dRelX_m, stEgoInfo.dRelY_m, stEgoInfo.dZ_m,
                        stEgoInfo.dLength_m, stEgoInfo.dWidth_m, stEgoInfo.dHeight_m,
                        stEgoInfo.dHeading_deg,iFlag, iIdx);

    // Object Info
    for(int32_t i  = 0; i < stVehicleInfo.iNumOfVehicle; i++)
    {
        visualization_msgs::Marker marker;
        EACH_VEHICLE_INFO &stVehicle = stVehicleInfo.stEachOfVehicle[i];
        if(stVehicleInfo.stEachOfVehicle[i].iTargetFlag == 1)
        {
            iFlag = 1;
        }
        else
        {
            iFlag = 2;
        }
        
        marker = DrawMarker(stVehicle.dRelX_m, stVehicle.dRelY_m, stVehicle.dZ_m,
                            stVehicle.dLength_m, stVehicle.dWidth_m, stVehicle.dHeight_m,
                            stVehicle.dHeading_deg, iFlag, i);
        marker_array.markers.push_back(marker);
    }
    //Publish
    pub_EgoVehicle.publish(marker);
    pub_Vehicle.publish(marker_array);
}

void AdusViewer::DrawPath()
{
    int32_t iFlag = 0;
    float64_t dHeading_rad = 0;
    // 1. Local Path
    if (fabs(stLTrajInfo.fParamD) < 2)
    {
        visualization_msgs::Marker LocalPath;
        LocalPath = DrawPathMarker(dHeading_rad, MAX_NUM_LTRAJ, iFlag);
        pub_LocalPath.publish(LocalPath);
    }

     // 2. Global Path
     iFlag = 1;
    if (stGpsInfo.dLat > 30 && stGpsInfo.dLon > 120)
    {
        visualization_msgs::Marker GlobalPath;
        dHeading_rad = (360.- stEgoInfo.dHeading_deg) * PI / 180.;
        GlobalPath = DrawPathMarker(dHeading_rad, stPathInfo.iNumOfPath, iFlag);
        pub_GlobalPath.publish(GlobalPath);
    }

}

void AdusViewer::DrawText()
{
    jsk_rviz_plugins::OverlayText overlay_text;
    overlay_text.action = jsk_rviz_plugins::OverlayText::ADD;
    overlay_text.width = 340;
    overlay_text.height = 380;
    overlay_text.left = 0;
    overlay_text.top = 240;
    overlay_text.text_size = 14;
    overlay_text.line_width = 2;
    overlay_text.font = "DejaVu Sans Mono";

    overlay_text.fg_color.r = 1.0;
    overlay_text.fg_color.g = 1.0;
    overlay_text.fg_color.b = 1.0;
    overlay_text.fg_color.a = 1.0;

    overlay_text.bg_color.r = 0.0;
    overlay_text.bg_color.g = 0.0;
    overlay_text.bg_color.b = 0.0;
    overlay_text.bg_color.a = 0.5;

    float64_t dDist = sqrt(stTargetInfo.fX_m  * stTargetInfo.fX_m  + (stTargetInfo.fY_m * stTargetInfo.fY_m ));

    std::ostringstream oss;
    oss << "[Ego Info]\n"
        << std::fixed << std::setprecision(1)
        << "Velocity: " << stEgoInfo.dSpeed_kph << "kph\n"
        << "Acceleration: " << stEgoInfo.dAcceleration_mpss << "mpss\n"
        << "Heading: " << stEgoInfo.dHeading_deg << "deg\n"
        << "Roll: " << stImuInfo.dRoll << "deg" << " "
        << "Pitch: " << stImuInfo.dPitch << "deg\n"
        // << "Yaw: " << stImuInfo.dYaw << "deg\n"
        << std::fixed << std::setprecision(12)
        << "Lat: " << stGpsInfo.dLat << "\n"
        << "Lon: " << stGpsInfo.dLon
        << "\n\n"
        << "[Target Info]\n"
        << std::fixed << std::setprecision(1)
        << "Dist:" << dDist << "m\n"
        << "TTC:" << fabs(stTargetInfo.fTTC) << "msec\n"
        << "Speed: " << fabs(stTargetInfo.fSpeedRel) << "mps\n\n"
        << "[Nearest Global Path]\n"
        << std::fixed << std::setprecision(12)
        << "Lat: " << stPathInfo.dNearestLat << "\n"
        << "Lon: " << stPathInfo.dNearestLon;

    overlay_text.text = oss.str();

    pub_Text.publish(overlay_text);
}

void AdusViewer::Image_callback(const sensor_msgs::CompressedImage::ConstPtr &msg)
{
    try
    {
        // 압축 이미지를 OpenCV 이미지로 디코딩
        cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (image.empty())
        {
            ROS_WARN("Empty image received");
            return;
        }

        // OpenCV 이미지를 ROS Image로 변환
        cv_bridge::CvImage cv_image;
        cv_image.image = image;
        cv_image.encoding = "bgr8";
        pub_Image.publish(cv_image.toImageMsg());
    }
    catch (cv::Exception &e)
    {
        ROS_ERROR("OpenCV Error: %s", e.what());
    }
}
