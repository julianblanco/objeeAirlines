#include <Arduino.h>
#ifndef _GPS_FUNCTIONS_H_
#define _GPS_FUNCTIONS_H_
#include "Navigation.h"
#define NUMBER_WAYPOINTS 2   
#define WAYPOINT_DIST_TOLERANE  2   // tolerance in meters to waypoint; once within this tolerance, will advance to the next waypoint





//void useInterrupt(boolean v);

void processGPS(float rawLat, float rawLong, float* myLat, float* myLong);

float convertDegMinToDecDeg (float degMin);

float courseToWaypoint(float lat1, float long1, float lat2, float long2);

void nextWaypoint(void);

float distanceToWaypoint(float lat1, float long1, float Lat2, float Long2);

void gpsrun(float* GpsHead, float rawLat, float rawLong, float* myLat, float* myLong);

void Update_GPS_Heading(float* GpsHead);


void navigate(float Lat1, float Long1, float Lat2, float Long2, float* target_heading, float* distance);

#endif // _GPS_FUNCTIONS_H_
