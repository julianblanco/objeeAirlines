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
float update_motors(float heading_error,float timed)
{
  float pResponse = heading_error * steeringKp;

  float steeringIntegral = 0;
  steeringIntegral = heading_error * steeringKi;
  static float steeringIntegralresponse = 0;
  steeringIntegralresponse += heading_error;
  if (steeringIntegral > 20)
  {
    steeringIntegral = 20;
  }
  if (steeringIntegral < -20)
  {
    steeringIntegral = -20;
  }

    if (steeringIntegral <3 && steeringIntegral > -3)
  {
    steeringIntegral = 0;
  }

  static float steeringError;
  static float steeringError1;
  static float steeringError2;
  static float steeringError3;
  static float steeringError4;

 // float derivativeterm = (((steeringError4 - steeringError3) / timed) + ((steeringError3 - steeringError2) / timed) + ((steeringError2 - steeringError1) / timed) + ((steeringError1 - error) / timed)) * (1 / 4);
 float derivativeterm =(steeringError1 - heading_error) / timed;
 
  steeringError1 = heading_error;
  steeringError2 = steeringError1;
  steeringError3 = steeringError2;
  steeringError4 = steeringError3;
  
  
  float derivativeerror = derivativeSetpoint - derivativeterm;
  float derivativeResponse = derivativeerror * steeringKd;
 // float response = response + steeringIntegralresponse + 1500;
  Serial.println(derivativeResponse);
  float response = 90+pResponse -steeringIntegralresponse+derivativeResponse;

  // Bound PWM output
  if ( response > 170)
    response = 170;

  else if (response < 20)
    response =20;

  return response;
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
