#include "GPS.h"

//===================================================================
//
//	Program:		GPS_functions.cpp
//
//	Author:			Julian Blanco
//	Date:			Sep 4 2015
//
//	Description:	Description
//
//===================================================================

void gpsrun(float* GpsHead, float rawLat, float rawLong, float* myLat, float* myLong)
{
  /**
//  
//  
//  
  */
  
  Update_GPS_Heading(GpsHead);
  //processGPS(rawLat, rawLong, myLat, myLong); // num, num, pointer, pointer
}

void navigate(float Lat1, float Long1, float Lat2, float Long2, float* target_heading, float* distance)
{
  *target_heading = courseToWaypoint(Lat1,Long1,Lat2,Long2);
  *distance = distanceToWaypoint(Lat1,Long1,Lat2,Long2);
}

void processGPS(float rawLat, float rawLong, float* myLat, float* myLong)
{

 *myLat = convertDegMinToDecDeg(rawLat);
 *myLong = convertDegMinToDecDeg(rawLong);
  
}


// processGPS(void)
//*******************************************************************************************************
// converts lat/long from Adafruit degree-minute format to decimal-degrees; requires <math.h> library
float convertDegMinToDecDeg (float degMin)
{
  double min = 0.0;
  double decDeg = 0.0;

  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);

  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );

  return decDeg;
}

//*******************************************************************************************************

float courseToWaypoint(float lat1, float long1, float lat2, float long2)
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  float dlon = radians(long2 - long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float a1 = sin(dlon) * cos(lat2);
  float a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  return degrees(a2);
}  // courseToWaypoint()
//*******************************************************************************************************


int nextWaypoint(int waypointNumber)
{
  waypointNumber++;
  if ( waypointNumber >= NUMBER_WAYPOINTS)    // last waypoint reached?
  {
    Serial.println(F("* LAST WAYPOINT *"));
  }
}  // nextWaypoint()

//*******************************************************************************************************

float distanceToWaypoint(float Lat1, float Long1, float Lat2, float Long2)
{
  float dist;
  float dLat = (float)(Lat2 - Lat1);                                    // difference of latitude in 1/10 000 000 degrees
  float dLon = (float)(Long2 - Long1) * cos(Lat1) ; //
  dist = sqrt(sq(dLat) + sq(dLon)) * 110575;
  
  return dist;
}

void Update_GPS_Heading(float* GpsHead)
{
  //insert filter here
  *GpsHead = *GpsHead;
}


