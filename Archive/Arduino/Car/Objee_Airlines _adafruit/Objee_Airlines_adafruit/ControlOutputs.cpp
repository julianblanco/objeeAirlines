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

void generate_control(float last_euler,float current_euler,float *current_heading,float Gps_heading, float *heading_error, float target_heading)
{
  
  //update_gyro_heading(float,float,float*
  
 // GPS_complement(current_heading,Gps_heading);
  *heading_error=control_output(*current_heading,target_heading);
 
  
}

int execute_control(float heading_error)
{
  int steer=update_motors(heading_error);
  return(steer);

}


// Calculate the current error between the current and desired headings
int control_output(float current_heading, float target_heading)
{
  float heading_error;
  // Calculate the heading error given the target and current headings
  heading_error = target_heading - current_heading;

  if ( heading_error >= 180)
    heading_error = heading_error - 360;

  if ( heading_error < -180)
    heading_error = heading_error + 360;

  return (heading_error); 
}


// Calculate control response and send control output to the motors
int update_motors(float heading_error)
{
  // Calculate left/right motor response (positive -> right)
  int response = (int)P_COEF * (int)heading_error;
  



int steer = response + 90;

// Bound PWM output
if(steer < 160)
  steer = 160;
else if (steer > 30)
  steer = 30;

return steer;

}


