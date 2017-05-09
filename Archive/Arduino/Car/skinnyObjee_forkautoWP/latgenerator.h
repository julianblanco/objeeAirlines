#ifndef _LATGENERATOR_H_
#define _LATGENERATOR_H_

#include <Arduino.h>
float latgenerator( float currentlat, float currentlong,float distance,float course );
float longgenerator( float currentlat, float currentlong,float distance,float course );
float createcourse(float course, int type);
float degtorad(float deg);

#endif // _LATGENERATOR_H_
