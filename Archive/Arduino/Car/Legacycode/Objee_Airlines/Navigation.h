#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

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
#endif // _NAVIGATION_H_
