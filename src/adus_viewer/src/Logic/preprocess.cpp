#include "preprocess.hpp"

void Preprocess::CalcRelativeCoordinate_Obj(VEHICLE_INFO *pstVehicleInfo, EGO_INFO *pstEgoInfo, TARGET_INFO *pstTargetInfo)
{
    float64_t dX_m = 0;
    float64_t dY_m = 0;
    float64_t dRelX_m = 0;
    float64_t dRelY_m = 0;
    float64_t dNewX_m = 0;
    float64_t dNewY_m = 0;

    float64_t dHeadingOffset_deg = (pstEgoInfo->dHeading_deg);
    float64_t dHeadingOffset_rad = (360.- dHeadingOffset_deg) * PI / 180.;

    if (pstVehicleInfo->iNumOfVehicle > 0)
    {
        for (int32_t i = 0; i < pstVehicleInfo->iNumOfVehicle; i++)
        {
            pstVehicleInfo->stEachOfVehicle[i].iTargetFlag = 0;
            dRelX_m = pstVehicleInfo->stEachOfVehicle[i].dX_m - pstEgoInfo->dX_m;
            dRelY_m =  pstVehicleInfo->stEachOfVehicle[i].dY_m - pstEgoInfo->dY_m;

            dNewX_m = dRelX_m * cos(dHeadingOffset_rad) - dRelY_m * sin(dHeadingOffset_rad);
            dNewY_m = dRelX_m * sin(dHeadingOffset_rad) + dRelY_m * cos(dHeadingOffset_rad);

            pstVehicleInfo->stEachOfVehicle[i].dRelX_m = dNewX_m;
            pstVehicleInfo->stEachOfVehicle[i].dRelY_m = dNewY_m;
            
          

            if(pstTargetInfo->iValidFlag == 1)
            {
                dX_m = (float64_t)pstTargetInfo->fX_m;   
                dY_m = (float64_t)pstTargetInfo->fY_m;   
                float64_t dDiffX_m = dX_m - dNewX_m;
                float64_t dDiffY_m = dY_m - dNewY_m;
                float64_t dDist = (dDiffX_m * dDiffX_m + dDiffY_m * dDiffY_m);
                if(dDist < 0.5 * 0.5)
                {
                    pstVehicleInfo->stEachOfVehicle[i].iTargetFlag = 1;
                    pstVehicleInfo->iTargetIdx = i;
                }
                else
                {
                    pstVehicleInfo->stEachOfVehicle[i].iTargetFlag = -1;
                      pstVehicleInfo->iTargetIdx = -1;
                }
            }
        }
       
        
    }
}

void Preprocess::CalcRelativeCoordinate_Path(GPS_INFO *pstGpsInfo, PATH_INFO *pstPathInfo, EGO_INFO *pstEgoInfo)
{
    float64_t dDist = 0.;
    float64_t dMinDist = FLT_MAX;
    float64_t dHeadingOffset_rad = 0 * (M_PI / 180.0);
    // GPS & Global Path
    if (pstGpsInfo->dLat > 0 && pstGpsInfo->dLon > 0)
    {
        POINT2D stUTMCoord_gps = {0, 0};
        stUTMCoord_gps = DegreeToUTM(pstGpsInfo->dLat, pstGpsInfo->dLon);
        pstGpsInfo->dX_m = stUTMCoord_gps.dX;
        pstGpsInfo->dY_m = stUTMCoord_gps.dY;
        for (int32_t iIdxI = 0; iIdxI < pstPathInfo->iNumOfPath; iIdxI++)
        {
            POINT2D stUTMCoord_path = {0, 0};
            float64_t dLat = pstPathInfo->dLat[iIdxI];
            float64_t dLon = pstPathInfo->dLon[iIdxI];
            stUTMCoord_path = DegreeToUTM(dLat, dLon);

            float64_t dX = stUTMCoord_path.dX - pstGpsInfo->dX_m;
            float64_t dY = stUTMCoord_path.dY - pstGpsInfo->dY_m;

            float64_t dNewX = 0.;
            float64_t dNewY = 0.;

            dNewX = dX * cos(dHeadingOffset_rad) - dY * sin(dHeadingOffset_rad);
            dNewY = dX * sin(dHeadingOffset_rad) + dY * cos(dHeadingOffset_rad);
        
            pstPathInfo->dX_m[iIdxI] = dNewX;
            pstPathInfo->dY_m[iIdxI] = dNewY;

            dDist = dNewX * dNewX + dNewY * dNewY;
            if(dDist < dMinDist)
            {
                dMinDist = dDist;
                pstPathInfo->dNearestLat = dLat;
                pstPathInfo->dNearestLon = dLon;
            }

        }
    }
}

POINT2D Preprocess::DegreeToUTM(float64_t dLat, float64_t dLon)
{
    POINT2D stUtmCoord = {0, 0};
    const float64_t sa = 6378137.0;                        // Semi-major axis
    const float64_t sb = 6356752.314245;                      // Semi-minor axis
    const float64_t e2 = sqrt((sa * sa - sb * sb)) / sb; // Eccentricity squared
    const float64_t c = (sa * sa) / sb;

    float64_t dLat_rad = dLat *  (M_PI / 180.0);
    float64_t dLon_rad = dLon *  (M_PI / 180.0);

    float64_t Huso = 52.0;
    float64_t S = (Huso * 6) - 183.;
    float64_t deltaS = dLon_rad - (S * (M_PI / 180.));
    
    // UTM calculations
    float64_t a = cos(dLat_rad) * sin(deltaS);
    float64_t epsilon = 0.5 * log((1 + a) / (1 - a));
    float64_t nu = atan(tan(dLat_rad) / cos(deltaS)) - dLat_rad;
    float64_t v = (c / sqrt(1 + e2 * e2 * pow(cos(dLat_rad), 2))) * 0.9996;
    float64_t ta = (e2 * e2 / 2) * pow(epsilon, 2) * pow(cos(dLat_rad), 2);
    float64_t a1 = sin(2 * dLat_rad);
    float64_t a2 = a1 * pow(cos(dLat_rad), 2);
    float64_t j2 = dLat_rad + (a1 / 2.0);
    float64_t j4 = (3.0 * j2 + a2) / 4.0;
    float64_t j6 = (5.0 * j4 + a2 * pow(cos(dLat_rad), 2)) / 3.0;
    float64_t alpha = (3.0 / 4.0) * e2 * e2;
    float64_t beta = (5.0 / 3.0) * pow(alpha, 2);
    float64_t gamma = (35.0 / 27.0) * pow(alpha, 3);
    float64_t Bm = 0.9996 * c * (dLat_rad - alpha * j2 + beta * j4 - gamma * j6);
    float64_t utm_x = epsilon * v * (1 + ta / 3.0) + 500000;
    float64_t utm_y = nu * v * (1 + ta) + Bm;

    // Adjust for southern hemisphere
    if (utm_y < 0) {
        utm_y += 9999999;
    }
    stUtmCoord.dX = utm_x;
    stUtmCoord.dY = utm_y;
    return stUtmCoord;
}