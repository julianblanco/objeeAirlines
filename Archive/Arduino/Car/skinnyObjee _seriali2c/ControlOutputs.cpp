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
  float P_COEF = .6;
  if(heading_error>15)
  {
   P_COEF = (heading_error)*(1/300) + .46666;
 }

 if(heading_error<-15)
 {
   P_COEF = (-1*heading_error)*(1/300) + .46666;
 }

  // Calculate left/right motor response (positive -> right)
//   if(heading_error>20)
//   {
//     P_COEF =.40;
//   }
//   if(heading_error<-20)
//   {
//     P_COEF=.40;
//   }

// if(heading_error>30)
//   {
//     P_COEF =.325;
//   }
//   if(heading_error<-30)
//   {
//     P_COEF=.325;
//   }

//   if(heading_error>50)
//   {
//     P_COEF =.25;
//   }
//   if(heading_error<-50)
//   {
//     P_COEF=.25;
//   }

//     if(heading_error>80)
//   {
//     P_COEF =.2;
//   }
//   if(heading_error<-80)
//   {
//     P_COEF=.2;

//   }

  /*
   if (abs(heading_error)>20)
   {
    P_COEF=heading_error*(1/400);
   }
  */
   int response = P_COEF * heading_error;


   int steer = response + 90;

// Bound PWM output
   if(steer > 135)
    steer = 135;
  else if (steer < 45)
    steer = 45;

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
