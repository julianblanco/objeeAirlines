#include "Arduino.h"
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

float rollPID(float error,int rollKp,int rollKi, int rollKd,float timed,int derivativeSetpoint) {
  float pResponse = error * rollKp;

  float rollIntegral = 0;
  rollIntegral = error * rollKi;
  static float rollIntegralresponse = 0;
  rollIntegralresponse += error;
  if (rollIntegral > 100)
  {
    rollIntegral = 100;
  }
  if (rollIntegral < -100)
  {
    rollIntegral = -100;
  }

  static float rollError;
  static float rollError1;
  static float rollError2;
  static float rollError3;
  static float rollError4;

  float derivativeterm = (((rollError4 - rollError3) / timed) + ((rollError3 - rollError2) / timed) + ((rollError2 - rollError1) / timed) + ((rollError1 - rollError) / timed)) * (1 / 4);
  rollError4 = rollError3;
  rollError3 = rollError2;
  rollError2 = rollError1;
  rollError1 = rollError;
  float derivativeerror = derivativeSetpoint - derivativeterm;
  float derivativeResponse = derivativeerror * rollKd;
 //float response = pResponse + rollIntegralresponse + 1500;
   float response = pResponse + 1500;

  // Bound PWM output
  if ( response > 1700)
    response = 1700;

  else if (response < 1300)
    response = 1300;

  return response;
}
float pitchPID(float error,int pitchKp,int pitchKi, int pitchKd,float timed,int derivativeSetpoint) {
  float pResponse = error * pitchKp;

  float pitchIntegral = 0;
  pitchIntegral = error * pitchKi;
  static float pitchIntegralresponse = 0;
  pitchIntegralresponse += error;
  if (pitchIntegral > 5000)
  {
    pitchIntegral = 5000;
  }
  if (pitchIntegral < -5000)
  {
    pitchIntegral = -5000;
  }

  static float pitchError;
  static float pitchError1;
  static float pitchError2;
  static float pitchError3;
  static float pitchError4;

  float derivativeterm = (((pitchError4 - pitchError3) / timed) + ((pitchError3 - pitchError2) / timed) + ((pitchError2 - pitchError1) / timed) + ((pitchError1 - pitchError) / timed)) * (1 / 4);
  pitchError4 = pitchError3;
  pitchError3 = pitchError2;
  pitchError2 = pitchError1;
  pitchError1 = pitchError;
  float derivativeerror = derivativeSetpoint - derivativeterm;
  float derivativeResponse = derivativeerror * pitchKd;
 // float response = response + pitchIntegralresponse + 1500;
  float response = 1500-pResponse ;

  // Bound PWM output
  if ( response > 1800)
    response = 1800;

  else if (response < 1100)
    response = 1100;

  return response;
}

float yawPID(float error,int yawKp,int yawKi, int yawKd,float timed,int derivativeSetpoint) {
  float pResponse = error * yawKp;
  float yawIntegral = 0;
  yawIntegral = error * yawKi;
  static float yawIntegralresponse = 0;
  yawIntegralresponse += yawIntegral;
  if (yawIntegral > 5000)
  {
    yawIntegral = 5000;
  }
  if (yawIntegral < -5000)
  {
    yawIntegral = -5000;
  }

  static float yawError;
  static float yawError1;
  static float yawError2;
  static float yawError3;
  static float yawError4;

  float derivativeterm = (((yawError4 - yawError3) / timed) + ((yawError3 - yawError2) / timed) + ((yawError2 - yawError1) / timed) + ((yawError1 - yawError) / timed)) * (1 / 4);
  yawError4 = yawError3;
  yawError3 = yawError2;
  yawError2 = yawError1;
  yawError1 = yawError;
  float derivativeerror = derivativeSetpoint - derivativeterm;
  float derivativeResponse = derivativeerror * yawKd;
  //float response = response + yawIntegralresponse + 1500;
    float response = 1500-pResponse  ;

  // Bound PWM output
  if ( response > 1800)
    response = 1800;

  else if (response < 1100)
    response = 1100;

  return response;
}
