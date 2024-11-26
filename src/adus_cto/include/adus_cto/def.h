#ifndef __DEF_H__
#define __DEF_H__

#include <ros/ros.h>

#define PI 3.14159265359
#define WHEEL_BASE      0.8

#define eps     2.22*pow(10, -16)
#define eps1    1*pow(10, -10)

// data parsing
#define WORD_LOW(x) (*(((unsigned char*)(&(x)))+0))
#define WORD_HIGH(x) (*(((unsigned char*)(&(x)))+1))

// polygon
#define SECTOR_GREEN 0.65   
#define SECTOR_RED 0.90
#define SECTOR_YELLOW 0.75

//se value ... speed max consider...
#define STEER_MAX 400
#define STEER_MIN -400

// speed 1806 .... 5kph
#define SPEED_MAX 1806
#define SPEED_MIN -1806

// #define SPEED_MAX 4800
// #define SPEED_MIN -4800

// hunter 2.0
// #define STEER_MAX 576
// #define STEER_MIN -576

// #define SPEED_MAX 1500
// #define SPEED_MIN -1500

// wp struct
typedef struct waypoints
{
    int index;
    double x_point = 0;
    double y_point = 0;
    double z_point = 0;
} waypoints;

typedef struct cur_pose
{
    double cur_x = 0;
    double cur_y = 0;
    double cur_z = 0;
    double cur_yaw = 0;
}cur_pose;

typedef struct can_tx_msg
{
	uint32_t id;
	uint8_t dlc;
	uint8_t data[8];
} can_tx_msg;

typedef struct can_rx_msg
{
    uint32_t id;
    unsigned char dlc;
    uint8_t data[8];
} can_rx_msg;

typedef struct h_se_tx_data
{
    uint8_t mode = 0;
    int16_t steer_cmd = 0;
    int16_t speed_cmd = 0;
}h_se_tx_data;

typedef struct h_se_rx_data
{
    std::string body_state = "Normal";
	std::string mode_state = "Standby";
    std::string parking_state = "Unlock";
    std::string comm_state = "Normal";

    uint16_t bat_volt = 0;

    int16_t speed_fdb = 0;
    int16_t steer_fdb = 0;

    int16_t f_rpm = 0;
    int16_t f_current = 0;
    int16_t f_pos_0 = 0;
    int16_t f_pos_1 = 0;
    int32_t f_pos = 0;

    int16_t rr_rpm = 0;
    int16_t rr_current = 0;
    int16_t rr_pos_0 = 0;
    int16_t rr_pos_1 = 0;
    int32_t rr_pos = 0;

    int16_t rl_rpm = 0;
    int16_t rl_current = 0;
    int16_t rl_pos_0 = 0;
    int16_t rl_pos_1 = 0;
    int32_t rl_pos = 0;
} h_se_rx_data;

#endif