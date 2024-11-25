#ifndef ADUS_DEFINE_HPP_
#define ADUS_DEFINE_HPP_
// Package Lib
#include <ros/ros.h>
#include <ros/package.h>

// Msg Lib
#include <nav_msgs/Path.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/CompressedImage.h>
#include <visualization_msgs/Marker.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_rviz_plugins/OverlayText.h>

#include "adss_msgs/DBZ03_Route.h"
#include "adss_msgs/DCD01_LTraj.h"
#include "adss_msgs/DEZ28_WayPoint.h"

#include "morai_msgs/GPSMessage.h"
#include "morai_msgs/ObjectStatusList.h"
#include "morai_msgs/EgoVehicleStatus.h"
// C/C++ Lib
#include <iostream>
#include <cmath>
#include <memory>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>
#include <sstream> 
#include <cv_bridge/cv_bridge.h> //Use Camera Topic
#include <opencv2/opencv.hpp> //Use Camera Topic


#define MORAI_HEADING_OFFSET 90.0
#define PI 3.141592653589793
#define DEG2RAD 0.017453292519943295
#define RAD2DEG 57.29577951308232

#define MAX_NUM_OBJECT 200
#define MAX_NUM_LTRAJ 100
#define MAX_NUM_PATH 10001

typedef float float32_t;
typedef double float64_t;

typedef struct EACH_VEHICLE_INFO_t
{
    int32_t iTargetFlag;
    float64_t dX_m;
    float64_t dY_m;
    float64_t dZ_m;
    float64_t dRelX_m;
    float64_t dRelY_m;
    float64_t dRelZ_m;
    float64_t dWidth_m;
    float64_t dLength_m;
    float64_t dHeight_m;
    float64_t dHeading_deg;
}EACH_VEHICLE_INFO;

typedef struct VEHICLE_INFO_t
{
    int32_t iTargetIdx;
    int32_t iNumOfVehicle;
    EACH_VEHICLE_INFO stEachOfVehicle[MAX_NUM_OBJECT];
}VEHICLE_INFO;

typedef struct EGO_INFO_t
{
    float64_t dX_m;
    float64_t dY_m;
    float64_t dZ_m;
    float64_t dRelX_m;
    float64_t dRelY_m;
    float64_t dRelZ_m;
    float64_t dWidth_m;
    float64_t dLength_m;
    float64_t dHeight_m;
    float64_t dHeading_deg;
}EGO_INFO;

typedef struct GPS_INFO_t
{
    float64_t dLon;
    float64_t dLat;
    float64_t dX_m;
    float64_t dY_m;
}GPS_INFO;

typedef struct LOCAL_TRAJ_INFO_t
{
    uint32_t uValid;
    float32_t fParamA;
    float32_t fParamB;
    float32_t fParamC;
    float32_t fParamD;
    float32_t fPathSpeedDist;
    float32_t fX_m[MAX_NUM_LTRAJ];
    float32_t fY_m[MAX_NUM_LTRAJ];
}LOCAL_TRAJ_INFO;

typedef struct PATH_INFO_t
{
    int32_t iNumOfPath;
    float64_t dLon[MAX_NUM_PATH];
    float64_t dLat[MAX_NUM_PATH];
    float64_t dX_m[MAX_NUM_PATH];
    float64_t dY_m[MAX_NUM_PATH];
}PATH_INFO;

typedef struct POINT2D_t
{
    float64_t dX;
    float64_t dY;

}POINT2D;


typedef struct POINT3D_t
{
    float64_t dX;
    float64_t dY;
    float64_t dZ;

}POINT3D;


typedef struct EulerAngle_t
{
    float64_t dRoll;
    float64_t dPitch;
    float64_t dYaw;

}EulerAngle;


#endif