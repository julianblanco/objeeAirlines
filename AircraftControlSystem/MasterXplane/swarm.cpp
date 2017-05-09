#include "variables.h"
#include "Navigation.h"
#include "tactics.h"
#include "Arduino.h"
#include "swarm.h"

#define traildist 20
void rightwingman()
{
 float tempheading =correctwrap((masterHeading+45));
 latgenerator(masterLat,masterLong,targetLat,targetLong,traildist,tempheading);
 float distanceError  =  distanceToWaypoint(myLat, myLong, masterLat, masterLong);
 float speedcorrection = 0.1 * distanceError;
 if(speedcorrection > 5)speedcorrection=5;
 if(speedcorrection < -5)speedcorrection=-5;
 desiredSpeed  = masterSpeed + speedcorrection;
}


void leftwingman()
{
	float tempheading =correctwrap((masterHeading+45));
 latgenerator(masterLat,masterLong,targetLat,targetLong,traildist,tempheading);

}

void followbehind()
{
	float tempheading =correctwrap((masterHeading+45));
	 latgenerator(masterLat,masterLong,targetLat,targetLong,traildist,tempheading);

}

void latgenerator( float currentlat, float currentlong,float& targetLat,float& targetLong,float distance,float course)
{
float degree = distance/111073;
course=degtorad(course);
targetLat=degree*cos(course);
targetLat=currentlat+targetLat;
targetLong=degree*sin(course);
targetLong=currentlong+targetLong;
}

float correctwrap(float course)
{

	if(course > 360)
	{
		course = course - 360;
	}
	if(course < 0 )
	{
		course =course +360;
	}

	return course;
}