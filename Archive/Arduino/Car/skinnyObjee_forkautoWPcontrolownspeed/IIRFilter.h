#ifndef _IIRFILTER_H_
#define _IIRFILTER_H_
#include <cmath>
#pragma once

using namespace std;
#define GPS_COEF .05

//added


float update_heading_estimate(float GPS_head, float gyro_current, float estimated_head, bool use_GPS);

// Fix any 360 degree wrap around that may occur while updating angular position
float correct_wrap(float current_heading);

// Find the shortest signed angular difference between a target and source angle
float angular_diff(float target_angle, float source_angle);

/*
// Header file for all navigation required data and functions

#define GPS_COEF .3

//struct _nav_data {

//};
//
//_nav_data update_gyro_heading(_nav_data nav_data);
//_nav_data GPS_complement(_nav_data nav_data);
//
//_nav_data control_output(_nav_data nav_data);


float correct_wrap(float current_heading);
void GPS_complement(float* current_heading, float Gps_heading);
void update_gyro_heading(float last_euler, float current_euler, float& current_heading);
void drive_gyro_only(float current_heading, float target_heading);

*/
#endif // _NAVIGr(float GPSinput, float steeringCompassIn, float gyroCurrent, float gyroLast, float inputPrecision);
