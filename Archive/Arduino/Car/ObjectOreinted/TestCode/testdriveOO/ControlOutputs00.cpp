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
  // Calculate left/right motor response (positive -> right)
  int response = (int)P_COEF * (int)heading_error;
  


  // Write responses to each motor
#ifdef ENABLE_MOTORS

#ifdef ANALOG_MOTORS
int PWM_left = velocidad - (int)response;
  int PWM_right = velocidad + (int)response;


  // Bound PWM values
  if (PWM_left > 255)
    PWM_left = 255;
  else if (PWM_left < 0)
    PWM_left = 0;

  if (PWM_right > 255)
    PWM_right = 255;
  else if (PWM_right < 0)
    PWM_right = 0;
analogWrite(LeftMotor,  PWM_left );
analogWrite(RightMotor, PWM_right);
#endif


#ifdef PWM_MOTOR
int steer = response + 1500;

// Bound PWM output
if(steer < 1200)
  steer = 1200;
else if (steer > 1800)
  steer = 1800;

//steer = map(steer,1200,1800,1800,1200);
steer = 1600;
return steer;
#endif  // PWM_MOTOR

#endif  // ENABLE_MOTORS
}


