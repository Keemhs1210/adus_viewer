#ifndef ADUS_INPUT_TOPIC_HPP_
#define ADUS_INPUT_TOPIC_HPP_

#include "define.hpp"
#include "Viewer/viewer.hpp"
#include "Logic/preprocess.hpp"

class InputTopic : public AdusViewer
{
public:
    InputTopic(ros::NodeHandle node, ros::NodeHandle private_nh,
               VEHICLE_INFO *pstVehicleInfo, EGO_INFO *pstEgoInfo,
               GPS_INFO *pstGpsInfo, PATH_INFO *pstPathInfo,
               LOCAL_TRAJ_INFO *pstLTrajInfo, TARGET_INFO *pstTargetInfo);

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

    VEHICLE_INFO *pstVehicleInfo_;
    EGO_INFO *pstEgoInfo_;
    GPS_INFO *pstGpsInfo_;
    PATH_INFO *pstPathInfo_;
    LOCAL_TRAJ_INFO *pstLTrajInfo_;
    TARGET_INFO *pstTargetInfo_;

    std::shared_ptr<Preprocess> preprocess_ptr;
    void ObjInfo_callback(const morai_msgs::ObjectStatusList &objInfo);
    void EgoInfo_callback(const morai_msgs::EgoVehicleStatus &EgoInfo);
    void GpsInfo_callback(const morai_msgs::GPSMessage &gpsInfo);
    void PathInfo_callback(const adss_msgs::DBZ03_Route &PathInfo);
    void LTajInfo_callback(const adss_msgs::DCD01_LTraj &LTrajInfo);
    void TargetInfo_callback(const adss_msgs::DCP11_TargetObject &TargetInfo);
};

#endif // ADUS_INPUT_TOPIC_HPP_
