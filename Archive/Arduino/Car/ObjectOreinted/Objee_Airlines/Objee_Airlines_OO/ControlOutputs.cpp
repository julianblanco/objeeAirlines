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
  update_gyro_heading(last_euler,current_euler,*current_heading);
 // GPS_complement(current_heading,Gps_heading);
  *heading_error=control_output(*current_heading,target_heading);
 
  
}

void execute_control(float heading_error)
{

  update_motors(heading_error);

}


// Calculate the current error between the current and desired headings
float control_output(float current_heading, float target_heading)
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
  
  

  // Write responses to each motor
#ifdef ENABLE_MOTORS

   // Calculate and bound control response
   int steer = (P_COEF * (int)heading_error) + 1500;
   if (steer < 1200)
     steer = 1200;
   else if (steer > 1800)
     steer = 1800;
return steer;

#endif  // ENABLE_MOTORS
}


