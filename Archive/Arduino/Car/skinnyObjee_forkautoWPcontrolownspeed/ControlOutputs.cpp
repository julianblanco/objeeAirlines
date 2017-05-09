#include "ControlOutputs.h"

//===================================================================
//
//	Program:		ControlOutputs.cpp
//
//	Author:			Julian Blanco
//	Date:			Sep 4 2015
//
//	Description:	Description
//
//===================================================================

// Calculate control response and send control output to the motors
float update_motors(float speed_error)
{
  float P_COEF =5;
  float motorspeed=0;

  motorspeed = speed_error*P_COEF;

  motorspeed = motorspeed + 1500;

// Bound PWM output
    if( motorspeed> 1800)
    motorspeed = 1800;
    else if (motorspeed < 1500)
    motorspeed = 1500;

return motorspeed;

}

float update_steering(float heading_error,float speed)
{
  float P_COEF =.6;
  // Calculate left/right motor response (positive -> right)
  if(heading_error>15)
  {
   P_COEF = (heading_error)*(1/300) + .46666;
  }

  if(heading_error<-15)
  {
   P_COEF = (-1*heading_error)*(1/300) + .46666;
  }

  P_COEF = P_COEF + P_COEF*(5/speed);

  if(P_COEF > .6)
    P_COEF =.6;

  int response = P_COEF * heading_error;
  

int steer = response + 90;

// Bound PWM output
    if(steer > 140)
    steer = 140;
    else if (steer < 40)
    steer = 40;

return steer;

}
float speedcalc(float heading_error,float distance2wp,float maxspeed)
{

float desiredspeed =0;

if(heading_error <0)
{
heading_error=heading_error*-1;
}

if(distance2wp < 12)
{
  desiredspeed = distance2wp - 4;
}
else 
{
  desiredspeed = maxspeed;
}

desiredspeed = desiredspeed- desiredspeed*(heading_error/180);

if (desiredspeed<3.5)
{
  desiredspeed=3.5;
}

return desiredspeed;
}
// Calculate the current error between the current and desired headings
float control_output(float current_heading, float target_heading)
{
// Calculate the heading error given the target and current headings
float heading_error = target_heading - current_heading;

if ( heading_error >= 180)
    heading_error = heading_error - 360;

if ( heading_error < -180)
    heading_error = heading_error + 360;

return (heading_error); 
}
