//craft.h
#ifndef _SWARM_H_
#define _SWARM_H_




void rightwingman();
void leftwingman();
void followbehind();
float correctwrap(float course);
void latgenerator( float currentlat, float currentlong, float& targetLat, float& targetLong, float distance,float course );

#endif