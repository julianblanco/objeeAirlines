#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

// Header file for all navigation required data and functions

//struct _nav_data {

//};
//
//_nav_data update_gyro_heading(_nav_data nav_data);
//_nav_data GPS_complement(_nav_data nav_data);
//
//_nav_data control_output(_nav_data nav_data);

float control_output(float current_heading, float target_heading);
void Navigation();
float distanceToWaypoint(float Lat1, float Long1, float Lat2, float Long2);
float courseToWaypoint(float lat1, float long1, float lat2, float long2);

#endif // _NAVIGATION_H_
