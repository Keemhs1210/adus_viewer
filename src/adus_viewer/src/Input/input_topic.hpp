#ifndef ADUS_INPUT_TOPIC_HPP_
#define ADUS_INPUT_TOPIC_HPP_

#include "define.hpp"
#include "Viewer/viewer.hpp"
#include "Logic/preprocess.hpp"
#include <tf/tf.h>


using namespace std;

class InputTopic : public AdusViewer
{
public:
    InputTopic(ros::NodeHandle node, ros::NodeHandle private_nh,
               VEHICLE_INFO *pstVehicleInfo, EGO_INFO *pstEgoInfo,
               GPS_INFO *pstGpsInfo, PATH_INFO *pstPathInfo,
               LOCAL_TRAJ_INFO *pstLTrajInfo, TARGET_INFO *pstTargetInfo,
               IMU_INFO *pstImuInfo);

    ~InputTopic() {};
    ros::NodeHandle node_;
    ros::NodeHandle private_nh_;


private:
    ros::Subscriber sub_ObjInfo;
    ros::Subscriber sub_EgoInfo;
    ros::Subscriber sub_GpsInfo;
    ros::Subscriber sub_TargetInfo;
    ros::Subscriber sub_PathInfo;
    ros::Subscriber sub_LTrajInfo;
    ros::Subscriber sub_ImuInfo;
    ros::Subscriber sub_RadarInfo;

    VEHICLE_INFO *pstVehicleInfo_;
    EGO_INFO *pstEgoInfo_;
    GPS_INFO *pstGpsInfo_;
    PATH_INFO *pstPathInfo_;
    LOCAL_TRAJ_INFO *pstLTrajInfo_;
    TARGET_INFO *pstTargetInfo_;
    IMU_INFO *pstImuInfo_;

    std::shared_ptr<Preprocess> preprocess_ptr;
    void ObjInfo_callback(const morai_msgs::ObjectStatusList::ConstPtr &objInfo);
    void EgoInfo_callback(const morai_msgs::EgoVehicleStatus::ConstPtr &EgoInfo);
    void GpsInfo_callback(const morai_msgs::GPSMessage::ConstPtr &gpsInfo);
    void PathInfo_callback(const adss_msgs::DBZ03_Route::ConstPtr &PathInfo);
    void LTajInfo_callback(const adss_msgs::DCD01_LTraj::ConstPtr &LTrajInfo);
    void TargetInfo_callback(const adss_msgs::DCP11_TargetObject::ConstPtr &TargetInfo);
    void ImuInfo_callback(const sensor_msgs::Imu::ConstPtr &ImuInfo);
};

#endif // ADUS_INPUT_TOPIC_HPP_
