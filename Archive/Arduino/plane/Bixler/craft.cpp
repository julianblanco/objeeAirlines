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
  //  trueHeading=update_heading_estimate(Gpsheading,currentEuler,trueHeading,use_GPS,gpsCoef);
    //////////////////////////////////////////////////////////////////////////////////////////////////
    ControlResponse(targetHeading, rollSetpoint, pitchSetpoint, trueHeading, rollInput, pitchInput);

orient_update=0;
}






  
    

                                                       //end loop



// Serial print to com port and xbee to matlab
////////////////////////////////////////////////////////////
void notify_user()
{
    
        // Serial.print(myLat,6);Serial.print(',');
        // Serial.print(myLong,6);Serial.print(',');
        // Serial.print(targetLat,6);Serial.print(',');
        // Serial.print(targetLong,6);Serial.print(',');
        Serial.print(targetHeading,4);Serial.print(',');
        // Serial.print(Gpsheading,4);Serial.print(',');
        // Serial.print(currentEuler,4);Serial.print(',');
         Serial.print(trueHeading,4);Serial.print(',');
         Serial.print(headingError,4);Serial.print(',');
        // Serial.print(GpsSpeed);Serial.print(',');
        // Serial.print(waypointNumber);Serial.print(',');
        // Serial.print(distanceToTarget,4);Serial.print(',');
        Serial.print(yawInput);Serial.print(',');
        Serial.print(rollInput);Serial.print(',');
        Serial.print(pitchInput);Serial.print(',');
        Serial.print(calibration);Serial.print(',');
        Serial.print(yawServoOutput);Serial.print(',');
        Serial.print(rollServoOutput);Serial.print(',');
        Serial.println(pitchServoOutput);
       
       // Serial.print(loopspeed);Serial.print(',');
      
      User_update=0;
}


void waypointing()
{
    if(distanceToTarget<5)
        {
        waypointNumber=waypointNumber+1;
        }

    targetLat = wplat[waypointNumber];
    targetLong = wplong[waypointNumber];
 distanceBetweenWaypoints=distanceToWaypoint(targetLat,targetLong,wplat[waypointNumber-1],wplong[waypointNumber-1]);
  courseBetweenWaypoints = courseToWaypoint(wplat[waypointNumber-1],wplong[waypointNumber-1],targetLat,targetLong);
    
}






void ControlResponse(float yawTargetValue, float rollTargetValue, float pitchTargetValue, float yawIn, float rollIn, float pitchIn) {
  // Calculate angular error
  float yawError =  control_output(yawIn, yawTargetValue);
  //Next Three lines bank the plane based on yaw error, and set satuaration point.
  rollTargetValue =1.3*yawError; 
  if (rollTargetValue > rollMaxAllowed) rollTargetValue=rollMaxAllowed;
  if (rollTargetValue < -1*rollMaxAllowed) rollTargetValue=-1*rollMaxAllowed;
    
  float rollError = rollTargetValue - rollIn;
  float pitchError = pitchTargetValue - pitchIn;
  

 
  yawResponse = yawPID(yawError);
  pitchResponse = pitchPID(pitchError);

  rollServoOutput = rollPID(rollError);

  rollIn=rollIn*(3.14159/180);

  yawServoOutput=(cos(rollIn)*yawResponse + sin(rollIn)*pitchResponse)+1500;
  pitchServoOutput=(sin(rollIn)*yawResponse + cos(rollIn)*pitchResponse)+1500;

// rollServoOutput = rollPID(rollError);
//   yawServoOutput = yawPID(yawError);
//   pitchServoOutput = pitchPID(pitchError);



  rollServoOutput = saturate(rollServoOutput,1900,1300);
   pitchServoOutput = saturate(pitchServoOutput,1900,1300);
    yawServoOutput = saturate(yawServoOutput,1900,1300);

}

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

float rollPID(float error) {
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

float pitchPID(float error) {
  float pResponse = -error * pitchKp;

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
  float response = pResponse ;

  // // Bound PWM output
  // if ( response > 1800)
  //   response = 1800;

  // else if (response < 1100)
  //   response = 1100;

  return response;
}

float yawPID(float error) {
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
    float response = -1*pResponse  ;

  // // Bound PWM output
  // if ( response > 1800)
  //   response = 1800;

  // else if (response < 1100)
  //   response = 1100;

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

  float newTargetHeading = (180/3.14159265)*targetHead+temp  ;

  if(newTargetHeading >= 360) newTargetHeading -= 360;
  else if(newTargetHeading < 0) newTargetHeading += 360;

  return newTargetHeading;
} // end crossTrackError

