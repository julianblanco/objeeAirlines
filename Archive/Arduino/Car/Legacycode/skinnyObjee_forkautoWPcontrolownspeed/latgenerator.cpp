#include "latgenerator.h"

/*function [ newlat,newlong ] = latgenerator( currentlat,currentlong,distance,course )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

degrees = distance/110575;
course=degtorad(course);
lat=degrees*cos(course);
long=degrees*sin(course);

newlat=currentlat+lat;
newlong=currentlong+long;

end
*/

float degtorad(float deg)
{
	float rad = deg*(3.14159265359/180);
	return rad;
}

float latgenerator( float currentlat, float currentlong,float distance,float course )
{
float newlat=0;
float degrees = distance/110575;
course=degtorad(course);
float lat=degrees*sin(course);

return newlat=currentlat+lat;


}

float longgenerator( float currentlat, float currentlong,float distance,float course )
{
float longi=0;

float degrees = distance/110575;
course=degtorad(course);

return longi=degrees*cos(course);

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