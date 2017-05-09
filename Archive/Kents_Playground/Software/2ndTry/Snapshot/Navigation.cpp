#include "Navigation.h"
#include "variables.h"
#include "Arduino.h"
//===================================================================
//
//	Program:		Navigation.cpp
//
//	Author:			Julian Blanco
//	Date:			Sep 4 2015
//
//	Description:	Description
//
//===================================================================

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

float distanceToWaypoint(float Lat1, float Long1, float Lat2, float Long2)
{
float dist;
float dLat = (float)(Lat2 - Lat1);                                    // difference of latitude in 1/10 000 000 degrees
float dLon = (float)(Long2 - Long1) * cos(Lat1) ; //
dist = sqrt(sq(dLat) + sq(dLon)) * 110575;

return dist;

}




void Navigation()
{

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    //Navigates
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // calculates the heading to the Waypoing
   
    //Navigates
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // calculates the heading to the Waypoing
    float XTerror= crossTrackError(distanceToTarget,courseBetweenWaypoints,oldHeading);

     if(waypointNumber>1 && XTerror > .25)
     {
       oldHeading = courseToWaypoint(myLat, myLong, targetLat, targetLong);
       targetHeading= crossTrackCorrection(XTerror,oldHeading );
     }
     
    else{
      targetHeading = courseToWaypoint(myLat, myLong, targetLat, targetLong);
    }



    //checks to see if we are close enough to the waypoint and 
    waypointing();
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}


void waypointing()
{
  
    //calculates the distance to the waypoint
    distanceToTarget = distanceToWaypoint(myLat, myLong, targetLat, targetLong);
    if(distanceToTarget<10)
        {
        waypointNumber=waypointNumber+1;
        // if(waypointNumber==2)
        // {
          
        // }
        }

    targetLat = wplat[waypointNumber];
    targetLong = wplong[waypointNumber];
    targetAlt =alt[waypointNumber];

  distanceBetweenWaypoints=distanceToWaypoint(targetLat,targetLong,wplat[waypointNumber-1],wplong[waypointNumber-1]);
  courseBetweenWaypoints = courseToWaypoint(wplat[waypointNumber-1],wplong[waypointNumber-1],targetLat,targetLong);

}