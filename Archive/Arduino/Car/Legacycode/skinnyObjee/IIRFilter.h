#ifndef _IIRFILTER_H_
#define _IIRFILTER_H_
#include <cmath>
#pragma once

using namespace std;
#define GPS_COEF .04

float IIRFilter(float GPSin, float GC, float steeringCompass, float precision);
float steeringCompassRefactor(float steeringCompass, float GPSin, float GL, float GC);

  // Fix any 360 degree wrap around that may occur while updating angular position
float correct_wrap(float current_heading);

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
