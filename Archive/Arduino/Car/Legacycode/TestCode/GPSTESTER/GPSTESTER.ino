   

#define NUMBER_WAYPOINTS 4   
#define WAYPOINT_DIST_TOLERANE  1   // tolerance in meters to waypoint; once within this tolerance, will advance to the next waypoint
#define GPSECHO  false
#include <waypointClass.h>    // custom class to manaage GPS waypoints
#include <math.h>  
  // current waypoint number; will run from 0 to (NUMBER_WAYPOINTS -1); start at -1 and gets initialized during setup()
waypointClass waypointList[NUMBER_WAYPOINTS] = {waypointClass( 41.372, -72.098), waypointClass(41.37261581, -72.09873962
 ), waypointClass(41.372, -72.098960), waypointClass( 41.37258148, -72.09924316
)};

float GPS_scaleLonDown;

int waypointNumber = -1;  
int i =0;
int satilites=0;
int originalDistanceToTarget;    // distance to original waypoing when we started navigating to it
int Address = 2;  //This slave is address number 2

float distanceToTarget;           // current distance to target (current waypoint)
float currentLat;
float currentLong;
float targetLat=41.370486  ;
float targetLong=-72.099156;
float myLat;
float myLong;
float targetHeading=270;
float fix=0;
float Gpsheading;
float SOG;




void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
nextWaypoint();
myLat= 41.37217712, -72.09899139 ;
myLong=-72.09899139;

}

void loop() {
  // put your main code here, to run repeatedly:

 
distanceToTarget = distanceToWaypoint(myLat,myLong,targetLat,targetLong);
targetHeading=courseToWaypoint(myLat,myLong,targetLat,targetLong);

 Serial.print("wp: ");
 Serial.println(waypointNumber);
 Serial.print("dist: ");
 Serial.println(distanceToTarget);
Serial.print("course: ");
Serial.println(targetHeading);
delay(5000);
}


float distanceToWaypoint(float myLat, float myLong, float targetLat, float targetLong)
{
 float dist;
 float dLat = (float)(targetLat - myLat);                                    // difference of latitude in 1/10 000 000 degrees
 float dLon = (float)(targetLong - myLong) * cos(myLat) ; //
 return dist = sqrt(sq(dLat) + sq(dLon)) * 110575;
}
float courseToWaypoint(float myLat, float long1, float targetHeading, float long2) 
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  float dlon = radians(long2-long1);
  myLat = radians(myLat);
  targetHeading = radians(targetHeading);
  float a1 = sin(dlon) * cos(targetHeading);
  float a2 = sin(myLat) * cos(targetHeading) * cos(dlon);
  a2 = cos(myLat) * sin(targetHeading) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  return degrees(a2);
}

void nextWaypoint(void)
{
  waypointNumber++;
  //targetLat = waypointList[waypointNumber].getLat();
//targetLong = waypointList[waypointNumber].getLong();
  
  if ((targetLat == 0 && targetLong == 0) || waypointNumber >= NUMBER_WAYPOINTS)    // last waypoint reached? 
    {
    
     Serial.println(F("* LAST WAYPOINT *"));
     
    }
    
   
}  // nextWaypoint()


