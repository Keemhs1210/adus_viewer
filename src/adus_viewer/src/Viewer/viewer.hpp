#ifndef ADUS_VIEWER_HPP_
#define ADUS_VIEWER_HPP_
#include "define.hpp"

class AdusViewer 
{
public:
    AdusViewer(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~AdusViewer(){};

    void AdusInit();
    ros::NodeHandle node_;
    ros::NodeHandle private_nh_;
private:
    void Visualization();
    void DrawVehicle();
    void DrawPath();
    void DrawText();
    void Image_callback(const sensor_msgs::CompressedImage::ConstPtr& msg);

    visualization_msgs::Marker DrawMarker(float64_t dX_m, float64_t dY_m, float64_t dZ_m,
                                      float64_t dLength_m, float64_t dWidth_m, float64_t dHeight_m, 
                                      float64_t dHeading_deg, int32_t Flag, int32_t iIdx);

    visualization_msgs::Marker DrawPathMarker(float64_t dHeading_rad, int32_t iNumOfPath, int32_t Flag);

    VEHICLE_INFO stVehicleInfo;
    EGO_INFO stEgoInfo;
    GPS_INFO stGpsInfo;
    PATH_INFO stPathInfo;
    LOCAL_TRAJ_INFO stLTrajInfo;

    int32_t iWidth = 0;
    int32_t iHeight = 0;
    ros::Subscriber sub_image;
    ros::Publisher pub_Vehicle;
    ros::Publisher pub_EgoVehicle;
    ros::Publisher pub_EgoHeading;
    ros::Publisher pub_GlobalPath;
    ros::Publisher pub_LocalPath; 
    ros::Publisher pub_Text;
    ros::Publisher pub_Image;
};

inline AdusViewer::AdusViewer(ros::NodeHandle node, ros::NodeHandle private_nh)
    : node_(node), private_nh_(private_nh)
{
    ROS_INFO("Start Adus Viewer");
    memset(&stVehicleInfo, 0, sizeof(VEHICLE_INFO));
    memset(&stEgoInfo, 0, sizeof(EGO_INFO));
    memset(&stPathInfo, 0, sizeof(PATH_INFO));
    memset(&stLTrajInfo, 0, sizeof(LOCAL_TRAJ_INFO));
}

inline visualization_msgs::Marker AdusViewer::DrawMarker(float64_t dX_m, float64_t dY_m, float64_t dZ_m,
                                      float64_t dLength_m, float64_t dWidth_m, float64_t dHeight_m, 
                                      float64_t dHeading_deg, int32_t iFlag, int32_t iIdx)
{
    float64_t dHeading_rad = 0;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.id = iIdx + 1;
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = dX_m;
    marker.pose.position.y = dY_m;
    marker.pose.position.z = dZ_m;

    // 크기 설정
    marker.scale.x = dLength_m;
    marker.scale.y = dWidth_m;
    marker.scale.z = dHeight_m;
    // Set Heading 
    marker.pose.orientation.x = 0.0; // No roll or pitch
    marker.pose.orientation.y = 0.0;

    if (iFlag == 0) // Ego
    {
        dHeading_rad = (90) * (M_PI / 180.);
        marker.pose.orientation.z = sin(dHeading_rad / 2.0);
        marker.pose.orientation.w = cos(dHeading_rad / 2.0);

        marker.ns = "ego_bounding_boxes";
        marker.color.r = 0.8f;
        marker.color.g = 0.8f;
        marker.color.b = 0.8f;
        marker.color.a = 0.8f;
    }
    else
    {
        dHeading_rad = (dHeading_deg) * (M_PI / 180.);
        marker.pose.orientation.z = sin(dHeading_rad / 2.0);
        marker.pose.orientation.w = cos(dHeading_rad / 2.0);

        marker.ns = "vehicle_bounding_boxes";
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.5f;

        if(iFlag == 1) //Target
        {
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.5f;
        }
        
    }
    return marker;
}

inline visualization_msgs::Marker AdusViewer::DrawPathMarker(float64_t dHeading_rad, int32_t iNumOfPath, int32_t iFlag)
{

    visualization_msgs::Marker Path;

    Path.header.frame_id = "map";
    Path.header.stamp = ros::Time::now();
    Path.id = iFlag;
    Path.type = visualization_msgs::Marker::LINE_STRIP;
    Path.action = visualization_msgs::Marker::ADD;

    Path.pose.orientation.x = 0.0;
    Path.pose.orientation.y = 0.0;
    Path.pose.orientation.z = sin(dHeading_rad / 2.0);
    Path.pose.orientation.w = cos(dHeading_rad / 2.0);

    if (iFlag == 0) //LTraj
    {
        Path.color.r = 255.0;
        Path.color.g = 144.0;
        Path.color.b = 30.0;
        Path.color.a = 0.6;
        Path.scale.x = 0.8;
    }
    else //Global Path
    {
        Path.color.r = 180.0;
        Path.color.g = 180.0;
        Path.color.b = 180.0;
        Path.color.a = 0.1;
        Path.scale.x = 5.;
    }

    for (int32_t iIdxI = 0; iIdxI < iNumOfPath; iIdxI++)
    {
        float64_t fX = 0;
        float64_t fY = 0;
        float64_t fZ = 0;
        geometry_msgs::Point pose;
        
        if (iFlag == 0)
        {
            fX = stLTrajInfo.fX_m[iIdxI];
            fY = stLTrajInfo.fY_m[iIdxI];
            fZ = 0;
        }
        else
        {
            fX = stPathInfo.dX_m[iIdxI];
            fY = stPathInfo.dY_m[iIdxI];
            fZ = -1;
        }
        pose.x = fX;
        pose.y = fY;
        pose.z = fZ;
        Path.points.push_back(pose);
    }

    return Path;
}

#endif