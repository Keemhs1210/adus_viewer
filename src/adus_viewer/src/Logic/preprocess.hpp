#ifndef ADUS_PREPROCSS_
#define ADUS_PREPROCSS_

#include "define.hpp"

class Preprocess
{
public:
    Preprocess(){};
    ~Preprocess() {}
    void CalcRelativeCoordinate_Obj(VEHICLE_INFO *pstVehicleInfo, EGO_INFO *pstEgoInfo, TARGET_INFO *pstTargetInfo);
    void CalcRelativeCoordinate_Path(GPS_INFO *pstGpsInfo, PATH_INFO *pstPathInfo, EGO_INFO *pstEgoInfo);

private:

    POINT2D DegreeToUTM(float64_t dLat, float64_t dLon);
};

#endif
