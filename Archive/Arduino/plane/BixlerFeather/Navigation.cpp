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
    waypointing(1);
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}


void waypointing(int state)
{
//state 1 normal waypointing
    if(state ==1){
    //calculates the distance to the waypoint
    distanceToTarget = distanceToWaypoint(myLat, myLong, targetLat, targetLong);
    if(distanceToTarget<10)
        {
        waypointNumber=waypointNumber+1;

        }

    targetLat = wplat[waypointNumber];
    targetLong = wplong[waypointNumber];
    targetAlt =alt[waypointNumber];

  distanceBetweenWaypoints=distanceToWaypoint(targetLat,targetLong,wplat[waypointNumber-1],wplong[waypointNumber-1]);
  courseBetweenWaypoints = courseToWaypoint(wplat[waypointNumber-1],wplong[waypointNumber-1],targetLat,targetLong);
}
if(state ==2)
{
  float vector =  correct_wrap(trueHeading + 315);
  targetLat = latgenerator(masterLat,masterLong,2,vector)
  targetLong = longgenerator(masterLat,masterLong,2,vector)
}

}//end waypointing()


float degtorad(float deg)
{
  float rad = deg*(3.14159265359/180);
  return rad;
}

float latgenerator( float currentlat, float currentlong,float distance,float course )
{
float newlat=0;
float degree = distance/110575;
course=degtorad(course);
float lat=degree*cos(course);
newlat=currentlat+lat;
return newlat;


}

float longgenerator( float currentlat, float currentlong,float distance,float course )
{
float longi=0;

float degree = distance/110575;
course=degtorad(course);
longi=degree*sin(course);
longi=currentlong+longi;
return longi;

}



//course = createcourse(course,1);
float createcourse(float course, int type)
{

if(type ==1)
{
//box
course=course+90;

}//end if

  
  if (course >= 360)
        course -= 360;
else if (course < 0)
        course += 360;

return(course);
}//end createcourse