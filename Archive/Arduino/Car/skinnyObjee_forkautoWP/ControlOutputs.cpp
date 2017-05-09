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
float update_motors(float heading_error)
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

  
  int response = P_COEF * heading_error;
  

int steer = response + 90;

// Bound PWM output
    if(steer > 140)
    steer = 140;
    else if (steer < 40)
    steer = 40;

return steer;

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
