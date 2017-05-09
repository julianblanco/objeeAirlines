//craft.cpp
#include "variables.h"
#include "craft.h"
#include "Arduino.h"

 
void UpdateOrientation()
{
       if(waypointNumber >1)
    {
         lasttargetLat = wplat[waypointNumber-1];
         lasttargetLong = wplong[waypointNumber-1];
         lastdistnace = distanceToWaypoint(myLat, myLong, lasttargetLat, lasttargetLong);
         // if(lastdistnace<8)
         // {
         //     new_GPS_data; = 0;
         // }
    }
// pitchSetpoint = 0;
  // counter_test++;
  // if(counter_test > 100)
  // {
  //   rollSetpoint = 30;
  // }

  ControlResponseWing(targetHeading, rollSetpoint, pitchSetpoint, trueHeading, rollInput, pitchInput);
}



// 2 surface plane
void ControlResponseWing(float &yawTargetValue, float &rollTargetValue, float &pitchTargetValue, float yawIn, float rollIn, float pitchIn) 
{
  float speedError = desiredSpeed - GpsSpeed;
  float rollError = rollTargetValue - rollIn;
  float pitchError= pitchTargetValue - pitchIn;
  if(rollInput>10)
  {
  pitch_offset = (7.778e-05)*(rollInput)*rollInput + (-3.333e-05)*rollInput + 0.0825;
  }
  else if(rollInput<-10)
  {
      pitch_offset = 0.0001*(rollInput)*rollInput + 0.001033*rollInput + 0.1025;
  }else{
    pitch_offset = 0.1;
  }
  //Calculate ServoOutputs based on PID control
   rollServoOutput = controlPID(rollError ,rollKp    ,rollKi ,rollKd ,userollKp,userollKi,userollKd,.5,400,400,0,rollAccum, rollErrorOld,rollInput);
   pitchServoOutput= controlPID(pitchError,pitchKp,pitchKi,pitchKd,usepitchKp,usepitchKi,usepitchKd,.5,400,400,pitch_offset,pitchAccum, pitchErrorOld,pitchInput);
   throtSetpoint= controlPID(speedError,.04,0,0,1,0,0,0,400,400,0.6,throtAcccum, throtlast,GpsSpeed);
   throtSetpoint=saturate(speedError,1,.5);     // this is actually the left servo command in microseconds 1000-2000
   rollServoOutput=saturate(rollServoOutput,1,-1);     // this is actually the left servo command in microseconds 1000-2000
   pitchServoOutput=saturate(pitchServoOutput,1,-1);   // this is actually the right servo command in microseconds 1000-2000
 }



float controlPID(float ErrorCurrent, float Kp, float Ki, float Kd, int useKP, int useKI, int useKD, float satuarationpoint, float satuarationPWMlow, float satuarationPWMhigh, float offset, float &Integralresponse, float &ErrorOld,float inputsignal) 
{



  //**************************** Porportional***********************************************
  //Calculate proportial response
  float pResponse = ErrorCurrent * Kp;

  //**************************** Integral***********************************************
  //Calculate Intergral response and add to accumulator: THINK ABOUT HOW TO RESET THE INTEGRAL RESPONSE
  Integralresponse += ErrorCurrent * Ki;

  if (Integralresponse > satuarationpoint) Integralresponse = satuarationpoint;
  if (Integralresponse < -1*satuarationpoint) Integralresponse = -1*satuarationpoint;


  //**************************** Deravitive***********************************************

  float timePID =62;                        // time elapsed per clock cycle; it works, but should be 100ms not 10. NEED TO HOOK UP TO OSCOPE

  //float derivativeterm = (((Error4 - Error3) / timePID) + ((Error3 - Error2) / timePID) + ((Error2 - Error1) / timePID) + ((Error1 - Error) / timePID)) * (1/4);

  //Calculate dx/dt
  float derivativeterm =(ErrorOld - inputsignal)/timePID;
  //pass current variable to error1 and overwrite previous value
  ErrorOld = inputsignal;
  //Calculate derivative Response
  float derivativeResponse = (derivativeSetpoint-derivativeterm)*Kd;      // tends towards derivativeSetpoint

  //**************************** Total***********************************************
  //Calculate total Response
  float response = pResponse + Integralresponse*useKI +derivativeResponse*useKD+offset;
  return response;
}


float saturate(float input,float upperbound,float lowerbound)
{
  if(input>upperbound) input=upperbound;
  if(input<lowerbound) input=lowerbound;
  return input;
}