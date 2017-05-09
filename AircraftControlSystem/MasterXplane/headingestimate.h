#ifndef _HEADINGESTIMATE_H_
#define _HEADINGESTIMATE_H_
#pragma once
#include "tactics.h"

void IIRFilter(void);

float update_heading_estimate(float GPS_head, float gyro_current, float estimated_head, bool use_GPS,float GPS_COEF);

void correctgyroheading(void);
// // Fix any 360 degree wrap around that may occur while updating angular position
// float correct_wrap(float current_heading);

// // Find the shortest signed angular difference between a target and source angle
// float angular_diff(float target_angle, float source_angle);

#endif 
// _NAVIGr(float GPSinput, float steeringCompassIn, float gyroCurrent, float gyroLast, float inputPrecision);