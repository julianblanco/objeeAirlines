//craft.cpp

#include "variables.h"
#include "craft.h"
#include "Arduino.h"

 
void UpdateOrientation()
{
    // Grab Gyro data
    ////////////////////////////////////////////////////////////////////////////////////////////////
    //grabs the gyro data from the sensor
   
    // Only use GPS heading data if there's a new fix and vehicle is travelling fast enough
   
    if(new_GPS_data > 0 && GpsSpeed > 2)
    {
        use_GPS = 1;
        new_GPS_data=new_GPS_data - 1;
    }
    else
    {
        use_GPS = 0;
    }

    if(waypointNumber >1)
    {
         lasttargetLat = wplat[waypointNumber-1];
         lasttargetLong = wplong[waypointNumber-1];
         lastdistnace = distanceToWaypoint(myLat, myLong, lasttargetLat, lasttargetLong);
         if(lastdistnace<8)
         {
             use_GPS = 0;
         }
    }


    // Update heading estimate with gyro and GPS data
    //////////////////////////////////////////////////////////////////////////////////////////////////

    //Added 0 here for temp override
    //ControlResponse(targetHeading, rollSetpoint, pitchSetpoint, trueHeading, rollInput, pitchInput);

ControlResponse(targetHeading, rollSetpoint, pitchSetpoint, trueHeading, rollInput, pitchInput);

}



// Serial print to com port and xbee to matlab
////////////////////////////////////////////////////////////
void notify_user()
{
    
        // Serial.print(myLat,6);Serial.print(',');
        // Serial.print(myLong,6);Serial.print(',');
        // Serial.print(targetLat,6);Serial.print(',');
        // Serial.print(targetLong,6);Serial.print(',');
        // Serial.print(targetHeading,4);Serial.print(',');
        // Serial.print(Gpsheading,4);Serial.print(',');
        // Serial.print(yawInput,4);Serial.print(',');
        // Serial.print(trueHeading,4);Serial.print(',');
        // Serial.print(headingError,4);Serial.print(',');
        // Serial.print(GpsSpeed);Serial.print(',');
        // Serial.print(waypointNumber);Serial.print(',');
        // Serial.print(distanceToTarget,4);Serial.print(',');
        // Serial.print(yawInput);Serial.print(',');
        // Serial.print(rollInput);Serial.print(',');
        // Serial.print(pitchInput);Serial.print(',');
        // Serial.print(calibration);Serial.print(',');
        // Serial.print(yawServoOutput);Serial.print(',');
        // Serial.print(rollServoOutput);Serial.print(',');
        // Serial.print(pitchServoOutput);Serial.print(',');

        Serial.print(altitudeControl(0,myAltitude,targetAlt,0,0));Serial.print(',');
        Serial.print(targetAlt,4);Serial.print(',');
        Serial.print(myAltitude,4);Serial.print(',');
        Serial.println(pressure,4);
}






//float controlPID(float error,float Kp, float Ki, float Kd, int useKP, int useKI,int useKD ,float satuarationpoint,float satuarationPWMlow,float satuarationPWMhigh,float offset) {


void ControlResponse(float yawTargetValue, float rollTargetValue, float pitchTargetValue, float yawIn, float rollIn, float pitchIn) {
  // Calculate angular error

  float yawError =  angular_diff( yawTargetValue,yawIn);
rollTargetValue = -.11*controlPID(yawError,yawKp,yawKi,yawKd,1,0,1,40,400,400,0);

  //Next Three lines bank the plane based on yaw error, and set satuaration point.


  //rollTargetValue=0;
  if (rollTargetValue > rollMaxAllowed) rollTargetValue=rollMaxAllowed;
  if (rollTargetValue < -1*rollMaxAllowed) rollTargetValue=-1*rollMaxAllowed;

  // Use shortest angle
  float rollError = rollTargetValue - rollIn;

pitchTargetValue=altitudeControl(0,myAltitude,targetAlt,0,0);
 //pitchTargetValue=15;
//pitchTargetValue=15;

  float pitchError = pitchTargetValue - pitchIn;

 
   pitchResponse = controlPID(pitchError,pitchKp,pitchKi,pitchKd,1,0,1,40,400,400,1500);
  rollServoOutput = controlPID(rollError,rollKp,rollKi,rollKd,1,0,1,40,400,400,1500);


  //rollIn=rollIn*(3.14159/180);

  yawServoOutput = controlPID(yawError,-1*yawKp,yawKi,yawKd,1,0,0,40,400,400,1500);
  pitchServoOutput = controlPID(pitchError,-1*pitchKp,pitchKi,pitchKd,1,0,0,40,400,400,1500);


 // yawServoOutput=(cos(rollIn)*yawResponse + .25*sin(rollIn)*pitchResponse)+1500;
  //pitchServoOutput=(.25*sin(rollIn)*yawResponse + cos(rollIn)*pitchResponse)+1500;

}
// Call at specific frequency
//float altitudeControl(float throttle, float altitude, float altitudeSetpoint, float airspeed, float desiredAirspeed){



float controlPID(float error,float Kp, float Ki, float Kd, int useKP, int useKI,int useKD ,float satuarationpoint,float satuarationPWMlow,float satuarationPWMhigh,float offset) {

  float pResponse = error * Kp;

  //fix so that mulltiple intergrals for each pid
  //use structure
  float Integral = 0;
  float timePID =10;

  Integral = error * Ki;

  static float Integralresponse = 0;
  Integralresponse += Integral;

  if (Integralresponse > satuarationpoint) Integralresponse = satuarationpoint;
  if (Integralresponse < -1*satuarationpoint) Integralresponse = -1*satuarationpoint;

  static float Error;
  static float Error1;
  static float Error2;
  static float Error3;
  static float Error4;

  //float derivativeterm = (((Error4 - Error3) / timePID) + ((Error3 - Error2) / timePID) + ((Error2 - Error1) / timePID) + ((Error1 - Error) / timePID)) * (1/4);
float derivativeterm =(Error1 -Error)/timePID;
  Error4 = Error3;
  Error3 = Error2;
  Error2 = Error1;
  Error1 = Error;

  float derivativeerror = derivativeSetpoint - derivativeterm;
  float derivativeResponse = derivativeerror * Kd;

  float response = pResponse*useKP + Integralresponse*useKI +derivativeResponse*useKD+offset;

  if (response > (offset+satuarationPWMlow)) {response  = offset+satuarationPWMlow;}
  if (response < (offset-satuarationPWMhigh)) {response  = offset-satuarationPWMhigh;}

  return response;
}


float crossTrackError(float distance2WP,float tracklegHead,float targetHead )
{
  
    // finds target headign to regain track
 
    //convert to radians for use with sin
  tracklegHead = (3.14159265/180) * tracklegHead;
  targetHead = (3.14159265/180) * targetHead;

    //compute heading error off trackline
  float deltaHeading=tracklegHead-targetHead;


    // crosstrack distance (positive if right of track)
  distanceXT = distance2WP * sin(deltaHeading);
return distanceXT;
}



float crossTrackCorrection(float distanceXT,float targetHead )
{
  float xtCoeff = -10;
  float temp = xtCoeff*distanceXT;

  if(temp>30) temp=30;
  if(temp<-30) temp=-30;

  float newTargetHeading = targetHead+temp  ;

  if(newTargetHeading >= 360) newTargetHeading -= 360;
  else if(newTargetHeading < 0) newTargetHeading += 360;

  return newTargetHeading;
} // end crossTrackError


// Call at specific frequency
float altitudeControl(float throttle, float altitude, float altitudeSetpoint, float airspeed, float desiredAirspeed){

///loat error,float Kp, float Ki, float Kd, int useKP, int useKI,int useKD ,float satuarationpoint,float satuarationPWM)
// Altitude error -> change throttle ... wait for airspeed to change ... airspeed error -> change pitch
// Find altitude error
float altitude_error = altitudeSetpoint-altitude ;

// Control throttle inversely proportionally to the altitude error
//throttle =controlPID(altitude_error,5,0,0,1,0,0,500,0,1000,0);

//throttle.writemicroseconds(throttle);?
// Increase or decrease pitch setpoint based on airspeed error
//float airspeed_error = airspeed-desiredAirspeed;

// Control pitch based on airspeed error

//float desiredPitch =controlPID(altitude_error,20,0,0,1,0,0,50,-12,20,0);
float desiredPitch = 3*altitude_error;
desiredPitch = saturate(desiredPitch,17,-10);
return desiredPitch;

}//end altitude control

float saturate(float input,float upperbound,float lowerbound){
  if(input>upperbound)
  {
    input=upperbound;
  }
  if(input<lowerbound)
  {
    input=lowerbound;
  }
  return input;
}