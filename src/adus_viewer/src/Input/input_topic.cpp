#include "input_topic.hpp"

// Callback Function List
void InputTopic::ObjInfo_callback(const morai_msgs::ObjectStatusList::ConstPtr &objInfo)
{
    int32_t iNumOfObj = 0;
    float64_t dX_m = 0;
    float64_t dY_m = 0;
    float64_t dZ_m = 0;
    float64_t dLength_m = 0;
    float64_t dWidth_m = 0;
    float64_t dHeight_m = 0;
    float64_t dHeading_deg = 0;
    float64_t dHeadingOffset_deg = MORAI_HEADING_OFFSET;
    for (int32_t i = 0; i < objInfo->num_of_npcs; i++)
    {
        dX_m = objInfo->npc_list[i].position.x;
        dY_m = objInfo->npc_list[i].position.y;
        dZ_m = objInfo->npc_list[i].position.z + 1.1;
        dLength_m = objInfo->npc_list[i].size.x;
        dWidth_m = objInfo->npc_list[i].size.y;
        dHeight_m = objInfo->npc_list[i].size.z;
        dHeading_deg = objInfo->npc_list[i].heading;

        pstVehicleInfo_->stEachOfVehicle[iNumOfObj].dX_m = dX_m;
        pstVehicleInfo_->stEachOfVehicle[iNumOfObj].dY_m = dY_m;
        pstVehicleInfo_->stEachOfVehicle[iNumOfObj].dZ_m = dZ_m;
        pstVehicleInfo_->stEachOfVehicle[iNumOfObj].dLength_m = dLength_m;
        pstVehicleInfo_->stEachOfVehicle[iNumOfObj].dWidth_m = dWidth_m;
        pstVehicleInfo_->stEachOfVehicle[iNumOfObj].dHeight_m = dHeight_m;
        pstVehicleInfo_->stEachOfVehicle[iNumOfObj].dHeading_deg = dHeading_deg + dHeadingOffset_deg;
        iNumOfObj++;
    }
        
    for (int32_t i = 0; i < objInfo->num_of_pedestrian; i++)
    {
        dX_m = objInfo->pedestrian_list[i].position.x;
        dY_m = objInfo->pedestrian_list[i].position.y;
        dZ_m = objInfo->pedestrian_list[i].position.z + 1.1;
        dLength_m = objInfo->pedestrian_list[i].size.x;
        dWidth_m = objInfo->pedestrian_list[i].size.y;
        dHeight_m = objInfo->pedestrian_list[i].size.z;
        dHeading_deg = objInfo->pedestrian_list[i].heading;

        pstVehicleInfo_->stEachOfVehicle[iNumOfObj].dX_m = dX_m;
        pstVehicleInfo_->stEachOfVehicle[iNumOfObj].dY_m = dY_m;
        pstVehicleInfo_->stEachOfVehicle[iNumOfObj].dZ_m = dZ_m;
        pstVehicleInfo_->stEachOfVehicle[iNumOfObj].dLength_m = dLength_m;
        pstVehicleInfo_->stEachOfVehicle[iNumOfObj].dWidth_m = dWidth_m;
        pstVehicleInfo_->stEachOfVehicle[iNumOfObj].dHeight_m = dHeight_m;
        pstVehicleInfo_->stEachOfVehicle[iNumOfObj].dHeading_deg = dHeading_deg + dHeadingOffset_deg;
        iNumOfObj++;
    }

            
    for (int32_t i = 0; i < objInfo->num_of_obstacle; i++)
    {
        dX_m = objInfo->obstacle_list[i].position.x;
        dY_m = objInfo->obstacle_list[i].position.y;
        dZ_m = objInfo->obstacle_list[i].position.z + 1.1;
        dLength_m = objInfo->obstacle_list[i].size.x;
        dWidth_m  = objInfo->obstacle_list[i].size.y;
        dHeight_m = objInfo->obstacle_list[i].size.z;
        dHeading_deg = objInfo->obstacle_list[i].heading;

        pstVehicleInfo_->stEachOfVehicle[iNumOfObj].dX_m = dX_m;
        pstVehicleInfo_->stEachOfVehicle[iNumOfObj].dY_m = dY_m;
        pstVehicleInfo_->stEachOfVehicle[iNumOfObj].dZ_m = dZ_m;
        pstVehicleInfo_->stEachOfVehicle[iNumOfObj].dLength_m = dLength_m;
        pstVehicleInfo_->stEachOfVehicle[iNumOfObj].dWidth_m = dWidth_m;
        pstVehicleInfo_->stEachOfVehicle[iNumOfObj].dHeight_m = dHeight_m;
        pstVehicleInfo_->stEachOfVehicle[iNumOfObj].dHeading_deg = dHeading_deg + dHeadingOffset_deg;
        iNumOfObj++;
    }
    pstVehicleInfo_->iNumOfVehicle = iNumOfObj;

}

void InputTopic::EgoInfo_callback(const morai_msgs::EgoVehicleStatus::ConstPtr &EgoInfo)
{
    float64_t dX_m = 0;
    float64_t dY_m = 0;
    float64_t dZ_m = 0;
    float64_t dSpeed_kph = 0;
    float64_t dAccel_mpss = 0;
    //IONIQ Car Size
    const float64_t dWidth_m = 1.82;
    const float64_t dLength_m = 4.467;
    const float64_t dHeight_m = 1.45;
    float64_t dHeading_deg = 0;
    float64_t dHeadingOffset_deg = MORAI_HEADING_OFFSET;
 
    dX_m = EgoInfo->position.x;
    dY_m = EgoInfo->position.y;
    dZ_m = EgoInfo->position.z + 1.1;
    dSpeed_kph = sqrt(EgoInfo->velocity.x * EgoInfo->velocity.x +
                      (EgoInfo->velocity.y * EgoInfo->velocity.y)) * 3.6;
    dAccel_mpss = sqrt(EgoInfo->acceleration.x * EgoInfo->acceleration.x +
                      (EgoInfo->acceleration.y * EgoInfo->acceleration.y));
    dHeading_deg = EgoInfo->heading + 180. + dHeadingOffset_deg;

    if(dHeading_deg> 360)
    {
        dHeading_deg -= 360.;
    }
    else if(dHeading_deg < 0)
    {
        dHeading_deg += 360.;
    }

    pstEgoInfo_->dX_m = dX_m;
    pstEgoInfo_->dY_m = dY_m;
    pstEgoInfo_->dZ_m = dZ_m;
    pstEgoInfo_->dRelX_m = 0.; //Local Coordi
    pstEgoInfo_->dRelY_m = 0.; //Local Coordi
    pstEgoInfo_->dRelZ_m = dZ_m;
    pstEgoInfo_->dLength_m = dLength_m;
    pstEgoInfo_->dWidth_m = dWidth_m;
    pstEgoInfo_->dHeight_m = dHeight_m;
    pstEgoInfo_->dHeading_deg = dHeading_deg;
    pstEgoInfo_->dSpeed_kph = dSpeed_kph;
    pstEgoInfo_->dAcceleration_mpss = dAccel_mpss;

    preprocess_ptr->CalcRelativeCoordinate_Obj(pstVehicleInfo_, pstEgoInfo_, pstTargetInfo_);
}

void InputTopic::GpsInfo_callback(const morai_msgs::GPSMessage::ConstPtr &gpsInfo)
{
    float64_t dLon = 0.;
    float64_t dLat = 0.;

    dLat = gpsInfo->latitude;
    dLon = gpsInfo->longitude;

    pstGpsInfo_->dLat = dLat;
    pstGpsInfo_->dLon = dLon;

    preprocess_ptr->CalcRelativeCoordinate_Path(pstGpsInfo_, pstPathInfo_, pstEgoInfo_);
}

void InputTopic::PathInfo_callback(const adss_msgs::DBZ03_Route::ConstPtr &PathInfo)
{
    int32_t iNumOfPath = 0;
    float64_t dLat = 0;
    float64_t dLon = 0;
    if (PathInfo->header.seq > 0)
    {
        for (int32_t i = 1; i < MAX_NUM_PATH; i++)
        {

            if (PathInfo->st_e_DEZ28_WayPoint_1x1.d_Lat_10000x1[i] < 30 &&
                PathInfo->st_e_DEZ28_WayPoint_1x1.d_Long_10000x1[i] < 120)
            {
                break;
            }
            dLat = PathInfo->st_e_DEZ28_WayPoint_1x1.d_Lat_10000x1[i];
            dLon = PathInfo->st_e_DEZ28_WayPoint_1x1.d_Long_10000x1[i];
            if (fabs(dLat) > 30 && fabs(dLon) > 120)
            {
                pstPathInfo_->dLat[iNumOfPath] = dLat;
                pstPathInfo_->dLon[iNumOfPath] = dLon;
                iNumOfPath++;
            }
        }
    }
    pstPathInfo_->iNumOfPath = iNumOfPath;
}

void InputTopic::LTajInfo_callback(const adss_msgs::DCD01_LTraj::ConstPtr &LTrajInfo)
{
    uint32_t uValid = 0;
    float32_t fParamA = 0;
    float32_t fParamB = 0;
    float32_t fParamC = 0;
    float32_t fParamD = 0;
    float32_t fPathSpeedDist = 0;

    uValid = LTrajInfo->e_val_Valid_1x1;
    fParamA = LTrajInfo->f_Coef3rd_1x1;
    fParamB = LTrajInfo->f_Coef2nd_1x1;
    fParamC = LTrajInfo->f_Coef1st_1x1;
    fParamD = LTrajInfo->f_CoefConst_1x1;
    fPathSpeedDist = LTrajInfo->f_PathSpeedDist_1x1;

    pstLTrajInfo_->uValid = uValid;
    pstLTrajInfo_->fParamA = fParamA;
    pstLTrajInfo_->fParamB = fParamB;
    pstLTrajInfo_->fParamC = fParamC;
    pstLTrajInfo_->fParamD = fParamD;
    pstLTrajInfo_->fPathSpeedDist = fPathSpeedDist;

    float32_t fX = 0;
    float32_t fY = 0;
    float32_t fNewX = 0;
    float32_t fNewY = 0;
    const float32_t fDist = 8;
    float32_t fStep = fDist / (float32_t)MAX_NUM_LTRAJ;
    float64_t dHeadingOffset_deg = MORAI_HEADING_OFFSET;
    float64_t dHeadingOffset_rad =  dHeadingOffset_deg * PI / 180.;


    for (int32_t iIdxI = 0; iIdxI < MAX_NUM_LTRAJ; iIdxI++)
    {
        fX = 0 + iIdxI * fStep;
        fY = fParamA * (fX * fX * fX) + fParamB * (fX * fX) + fParamC * fX + fParamD;

        fNewX = fX * cos(dHeadingOffset_rad) - fY * sin(dHeadingOffset_rad);
        fNewY = fX * sin(dHeadingOffset_rad) + fY * cos(dHeadingOffset_rad);
        pstLTrajInfo_->fX_m[iIdxI] = fNewX;
        pstLTrajInfo_->fY_m[iIdxI] = fNewY;
    }
}

void InputTopic::TargetInfo_callback(const adss_msgs::DCP11_TargetObject::ConstPtr &TargetInfo)
{
    int32_t iValid = 0;
    float32_t fX_m = 0;
    float32_t fY_m = 0;
    float32_t fTTC = 0.;
    float32_t fSpeedRel = 0;

    iValid = TargetInfo->e_val_Validity_1x1;
    pstTargetInfo_->iValidFlag = iValid;
    if(iValid != 0)
    {
        fX_m = TargetInfo->f_Pose_X_1x1;
        fY_m = TargetInfo->f_Pose_Y_1x1;
        fTTC = TargetInfo->f_TTC_1x1;
        fSpeedRel = TargetInfo->f_SpeedRel_1x1;

        pstTargetInfo_->fX_m = fY_m;
        pstTargetInfo_->fY_m = fX_m;
        pstTargetInfo_->fTTC = fTTC;
        pstTargetInfo_->fSpeedRel = fSpeedRel;
    }
    else{

        pstTargetInfo_->fX_m = 0;
        pstTargetInfo_->fY_m = 0;
        pstTargetInfo_->fTTC = 0;
        pstTargetInfo_->fSpeedRel = 0;
    }
}


void InputTopic::ImuInfo_callback(const sensor_msgs::Imu::ConstPtr &ImuInfo)
{
    float64_t dQuat_x = 0.;
    float64_t dQuat_y = 0.;
    float64_t dQuat_z = 0.;
    float64_t dQuat_w = 0.;
    float64_t dRoll = 0.;
    float64_t dPitch = 0.;
    float64_t dYaw = 0.;

    dQuat_x = ImuInfo->orientation.x;
    dQuat_y = ImuInfo->orientation.y;
    dQuat_z = ImuInfo->orientation.z;
    dQuat_w = ImuInfo->orientation.w;
    tf::Quaternion quaternion(dQuat_x, dQuat_y, dQuat_z, dQuat_w);

    tf::Matrix3x3(quaternion).getRPY(dRoll, dPitch, dYaw);

    pstImuInfo_->dRoll = dRoll * (180. / M_PI);
    pstImuInfo_->dPitch = dPitch * (180. / M_PI);
    pstImuInfo_->dYaw = dYaw * (180. / M_PI);
    pstImuInfo_->dYaw  = pstImuInfo_->dYaw + 180 + 90.;
    if (pstImuInfo_->dYaw > 360)
    {
        pstImuInfo_->dYaw -= 360.;
    }
    else if (pstImuInfo_->dYaw < 0)
    {
        pstImuInfo_->dYaw += 360.;
    }
}


InputTopic::InputTopic(ros::NodeHandle node, ros::NodeHandle private_nh,
                       VEHICLE_INFO *pstVehicleInfo, EGO_INFO *pstEgoInfo,
                       GPS_INFO *pstGpsInfo, PATH_INFO *pstPathInfo,
                       LOCAL_TRAJ_INFO *pstLTrajInfo, TARGET_INFO *pstTargetInfo, 
                       IMU_INFO_t *pstImuInfo)
    : AdusViewer(node, private_nh)
{
    ROS_INFO("Receive the Ros Topic");
    preprocess_ptr = std::make_shared<Preprocess>();

    // Get Structure Data
    pstVehicleInfo_ = pstVehicleInfo;
    pstEgoInfo_ = pstEgoInfo;
    pstGpsInfo_ = pstGpsInfo;
    pstPathInfo_ = pstPathInfo;
    pstLTrajInfo_ = pstLTrajInfo;
    pstTargetInfo_ = pstTargetInfo;
    pstImuInfo_ = pstImuInfo;

    sub_ObjInfo = node.subscribe("Object_topic", 1, &InputTopic::ObjInfo_callback, this);
    sub_EgoInfo = node.subscribe("Ego_topic", 1, &InputTopic::EgoInfo_callback, this);
    sub_GpsInfo = node.subscribe("gps", 1, &InputTopic::GpsInfo_callback, this);
    sub_PathInfo = node.subscribe("choice_path", 1, &InputTopic::PathInfo_callback, this);
    sub_LTrajInfo = node.subscribe("LTraj", 1, &InputTopic::LTajInfo_callback, this);
    sub_TargetInfo = node.subscribe("target_obj", 1, &InputTopic::TargetInfo_callback, this);
    sub_ImuInfo = node.subscribe("imu", 1, &InputTopic::ImuInfo_callback, this);
}

