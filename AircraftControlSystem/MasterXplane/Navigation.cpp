#include "Navigation.h"
#include "variables.h"
#include "swarm.h"
#include "tactics.h"
#include "Arduino.h"

#define waypointmindistance 50
//===================================================================
//
//  Program:    Navigation.cpp
//
//  Author:     Julian Blanco
//  Date:     Sep 4 2015
//
//  Description:  Description
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
dist = sqrt(sq(dLat) + sq(dLon)) * 110312;

return dist;

}

void Navigation()
{
  int are_waypoint =1;
    if(are_waypoint == 1)
    {
    float XTerror= crossTrackError(distanceToTarget,courseBetweenWaypoints,oldHeading);

     if(waypointNumber>1 && XTerror >  1)
     {
       oldHeading = courseToWaypoint(myLat, myLong, targetLat, targetLong);
       targetHeading= crossTrackCorrection(XTerror,oldHeading,distanceToTarget);
     }
     
    else{
      targetHeading = courseToWaypoint(myLat, myLong, targetLat, targetLong);
    }



    //checks to see if we are close enough to the waypoint and 
    waypointing();
  }
  else //swarming
  {
   
   // if(aircraftnumber==1)
 
    rightwingman();
   // if(aircraftnumber==2)leftwingman();

   targetHeading = courseToWaypoint(myLat, myLong, targetLat, targetLong);
   // desiredspeed = cruisespeed;
  }

}


// void Navigation()
// {
//   int are_waypoint =1;
//     if(are_waypoint == 1)
//     {
//     float XTerror= crossTrackError(distanceToTarget,courseBetweenWaypoints,oldHeading);

//      if(waypointNumber>1 && XTerror >  1)
//      {
//        oldHeading = courseToWaypoint(kalmanLat, kalmanLong, targetLat, targetLong);
//        targetHeading= crossTrackCorrection(XTerror,oldHeading,distanceToTarget);
//      }
     
//     else{
//       targetHeading = courseToWaypoint(kalmanLat, kalmanLong, targetLat, targetLong);
//     }



//     //checks to see if we are close enough to the waypoint and 
//     waypointing();
//   }
//   else //swarming
//   {
   
//    // if(aircraftnumber==1)
 
//     rightwingman();
//    // if(aircraftnumber==2)leftwingman();

//    targetHeading = courseToWaypoint(kalmanLat, kalmanLong, targetLat, targetLong);
//    // desiredspeed = cruisespeed;
//   }

// }

void waypointing()
{

    //calculates the distance to the waypoint
    
   
    static float tiemp=0;
    // float timep = ((millis())/1000);
    distancetraveled =((tiempo-lasttime)/1000)*(GpsSpeed*0.44704);

    // latgen(templat,templong,projectedLat,projectedLong,distancetraveled,trueHeading);
    // lasttime = tiempo;
    // templat = projectedLat;
    // templong = projectedLong;

    distanceToTarget = distanceToWaypoint(myLat, myLong, targetLat, targetLong);
    if(distanceToTarget<waypointmindistance)
        {
          waypointNumber=waypointNumber+1;

          if(waypointNumber==numOfWaypoints)
          {
            waypointNumber=0;
          }
        }

    targetLat = wplat[waypointNumber];
    targetLong = wplong[waypointNumber];
    targetAlt =alt[waypointNumber];

  distanceBetweenWaypoints=distanceToWaypoint(targetLat,targetLong,wplat[waypointNumber-1],wplong[waypointNumber-1]);
  courseBetweenWaypoints = courseToWaypoint(wplat[waypointNumber-1],wplong[waypointNumber-1],targetLat,targetLong);


}//end waypointing()
// void waypointing()
// {

//     //calculates the distance to the waypoint
    
   
//     static float tiemp=0;
//     // float timep = ((millis())/1000);
//     distancetraveled =((tiempo-lasttime)/1000)*(GpsSpeed*0.44704);

//     // latgen(templat,templong,projectedLat,projectedLong,distancetraveled,trueHeading);
//     // lasttime = tiempo;
//     // templat = projectedLat;
//     // templong = projectedLong;

//     distanceToTarget = distanceToWaypoint(kalmanLat, kalmanLong, targetLat, targetLong);
//     if(distanceToTarget<waypointmindistance)
//         {
//           waypointNumber=waypointNumber+1;

//           if(waypointNumber==numOfWaypoints)
//           {
//             waypointNumber=0;
//           }
//         }

//     targetLat = wplat[waypointNumber];
//     targetLong = wplong[waypointNumber];
//     targetAlt =alt[waypointNumber];

//   distanceBetweenWaypoints=distanceToWaypoint(targetLat,targetLong,wplat[waypointNumber-1],wplong[waypointNumber-1]);
//   courseBetweenWaypoints = courseToWaypoint(wplat[waypointNumber-1],wplong[waypointNumber-1],targetLat,targetLong);


// }//end waypointing()

void latgen( float currentlat, float currentlong,float& targetLat,float& targetLong,float distance,float course)
{
float degree = distance/111073;
course=degtorad(course);
targetLat=degree*cos(course);
targetLat=currentlat+targetLat;
targetLong=degree*sin(course);
targetLong=currentlong+targetLong;
}


void wpadd(float newwplat,float newwplong){
  numOfWaypoints+=1;
  wplat[numOfWaypoints]=newwplat;
  wplong[numOfWaypoints]=newwplong;

}

float degtorad(float deg)
{
  float rad = deg*(3.14159265359/180);
  return rad;
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