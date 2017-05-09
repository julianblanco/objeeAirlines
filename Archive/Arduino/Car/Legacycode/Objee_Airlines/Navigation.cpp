#include "Navigation.h"
//#include "Gyro.h"
#include "ControlOutputs.h"

//===================================================================
//
//	Program:		Navigation.cpp
//
//	Author:			Julian Blanco
//	Date:			Sep 4 2015
//
//	Description:	Description
//
//===================================================================

 
 
void update_gyro_heading(float last_euler, float current_euler, float& current_heading)
{
  // Move Euler reading from last loop
 
 
  // Update current heading given the change in the gyro angle since last polled
  current_heading = current_heading + (last_euler - current_euler); // update state estimation

  // Correct 360 deg wrap around
  current_heading = correct_wrap(current_heading);
  last_euler = current_euler;
}



// Use the GPS reported true heading to anchor the gyro estimated heading to a true heading
void GPS_complement(float* current_heading, float Gps_heading)
{
  float error = Gps_heading - *current_heading;
  error=abs(error);
  if(error>180)
  {
    *current_heading=(*current_heading+360);
    *current_heading = GPS_COEF * Gps_heading + (1 - GPS_COEF) * (*current_heading);
    *current_heading= *current_heading -360;
    *current_heading = correct_wrap(*current_heading);
  }
  else{
  // Complement GPS data with regularly updated gyro data
  *current_heading = GPS_COEF * Gps_heading + (1 - GPS_COEF) * (*current_heading);
  // Correct 360 deg wrap around
  *current_heading = correct_wrap(*current_heading);
  }
//  return (current_heading);
}



// Fix any 360 degree wrap around that may occur while updating angular position
float correct_wrap(float current_heading)
{

  // Correct 360 deg wrap around
  if ( current_heading > 360)
    current_heading = current_heading - 360;

  if (current_heading < 0)
    current_heading = current_heading + 360;

  return (current_heading);
}



void drive_gyro_only(float current_heading, float target_heading)
{
  
  float headerror=control_output(current_heading,target_heading);
   update_motors(headerror);
   
}

