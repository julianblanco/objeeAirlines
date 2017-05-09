#include "variables.h"
#include "Navigation.h"
#include "tactics.h"
#include "Arduino.h"

#define traildist 100
// void rightwingman(float masterLat, float masterLong, float& targetLat,float& targetLong,int currentspeed, int&targetSpeed, int desiredDistance)
// {

//  latgenerator(masterLat,masterLong,targetLat,targetLong,traildist,correctwrap(course+45));

// }


// void leftwingman(float masterLat, float masterLong, float& targetLat,float& targetLong,int currentspeed, int&targetSpeed, int desiredDistance)
// {
//  latgenerator(masterLat,masterLong,targetLat,targetLong,traildist,correctwrap(course-45));

// }

// void followbehind(float masterLat, float masterLong, float& targetLat,float& targetLong,int currentspeed, int&targetSpeed, int desiredDistance)
// {
// 	 latgenerator(masterLat,masterLong,targetLat,targetLong,traildist,correctwrap(course+180));

// }

// void latgenerator( float currentlat, float currentlong, float& targetLat, float& targetLong, float distance,float course )
// {
// float degree = distance/111073;
// course=degtorad(course);
// targetLat=degree*cos(course);
// targetLat=currentlat+targetLat;
// targetLong=degree*sin(course);
// targetLong=currentlong+targetLong;
// }

// int correctwrap(int course)
// {

// 	if(course > 360)
// 	{
// 		course = course - 360;
// 	}
// 	if(course < 0 )
// 	{
// 		course =course +360;
// 	}

// 	return course;
// }