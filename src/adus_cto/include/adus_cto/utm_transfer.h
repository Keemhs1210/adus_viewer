#ifndef __UTM_TRANSFER_H__
#define __UTM_TRANSFER_H__

#include "utm_transfer.h"

#include <ros/ros.h>
#include <termios.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define FLOAT_32

// #ifdef FLOAT_64
#define FLOAT double
#define SIN sin
#define COS cos
#define TAN tan
#define POW pow
#define SQRT sqrt
#define FLOOR floor

// #else
// #ifdef FLOAT_32
// #define FLOAT float
// #define SIN sinf
// #define COS cosf
// #define TAN tanf
// #define POW powf
// #define SQRT sqrtf
// #define FLOOR floorf

// #endif
// #endif

#define pi 3.14159265358979
#define sm_a 6378137.0
#define sm_b 6356752.314
#define sm_EccSquared 6.69437999013e-03
#define UTMScaleFactor 0.9996

class utm_transfer
{
private:

public:
    utm_transfer(/* args */);
    ~utm_transfer();    
    
public:
    FLOAT DegToRad(FLOAT deg);
    FLOAT RadToDeg(FLOAT rad);
    FLOAT ArcLengthOfMeridian (FLOAT phi);
    FLOAT UTMCentralMeridian(int zone);
    FLOAT FootpointLatitude(FLOAT y);
    void MapLatLonToXY (FLOAT phi, FLOAT lambda, FLOAT lambda0, FLOAT &x, FLOAT &y);
    void MapXYToLatLon (FLOAT x, FLOAT y, FLOAT lambda0, FLOAT& phi, FLOAT& lambda);
    int LatLonToUTMXY (FLOAT lat, FLOAT lon, int zone, FLOAT& x, FLOAT& y);
    void UTMXYToLatLon (FLOAT x, FLOAT y, int zone, bool southhemi, FLOAT& lat, FLOAT& lon);

};

#endif