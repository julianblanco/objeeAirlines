
#include "variables.h"
#include "craftcontrol.h"
#include "tactics.h"
#include "Arduino.h"



void controlTactics(void)
{
 float headingError =  angular_diff( 0,trueHeading);
  rollSetpoint = controlPID(headingError,headingKp,headingKi,headingKd,1,0,1,40,40,40,0,headingAccum, headingErrorOld,headingError);
   //Saturation point on roll rollMaxAllowed set in header
  if (rollSetpoint > rollMaxAllowed) rollSetpoint=rollMaxAllowed;
  if (rollSetpoint < -1*rollMaxAllowed) rollSetpoint=-1*rollMaxAllowed;
  rollTargetValue = rollSetpoint;
  altitudeControl(throtSetpoint,pitchSetpoint,myAltitude,targetAlt,GpsSpeed,5);


}



float crossTrackError(float distance2WP,float tracklegHead,float targetHead)
{
  //convert to radians for use with sin
  tracklegHead = (3.14159265/180) * tracklegHead;
  targetHead = (3.14159265/180) * targetHead;

  //compute heading error off trackline
  float deltaHeading=tracklegHead-targetHead;

  // crosstrack distance (positive if right of track)
  distanceXT = distance2WP * sin(deltaHeading);
  
  return distanceXT;
}



float crossTrackCorrection(float distanceXT,float targetHead,float distance2WP)
{
  float xtCoeff = -100;                // based on experimental data from the autonomous car
  float temp = (xtCoeff*distanceXT)/distance2WP;

  if(temp>30) temp=30;                // maximum allowable correction
  if(temp<-30) temp=-30;

  float newTargetHeading = targetHead+temp  ;

  if(newTargetHeading >= 360) newTargetHeading -= 360;
  else if(newTargetHeading < 0) newTargetHeading += 360;

  return newTargetHeading;
} // end crossTrackError


// Call at specific frequency
void altitudeControl(float &throttle,float &desiredPitch, float altitude, float altitudeSetpoint, float airspeed, float desiredAirspeed)
{
  // Altitude error -> change throttle ... wait for airspeed to change ... airspeed error -> change pitch
  // Find altitude error
  float altitude_error = altitudeSetpoint-altitude ;
   //float altitude_error = 100-altitude ;


  // Increase or decrease pitch setpoint based on airspeed error
  float airspeed_error = desiredAirspeed-airspeed;

  // Control pitch based on airspeed error
 
    throttle = controlPID(airspeed_error,10,1,0,1,1,0,1000,400,400,1500, airspeedAccum, airspeedErrorOld,airspeed);
  

  throttle= saturate(throttle,2200,1000);
  desiredPitch =controlPID(altitude_error,.2,.01,.9,1,.001,1,5,20,0,0, altitudeAccum, altitudeErrorOld,altitude);      // 20 degree pitch at 5 feet of error
 desiredPitch = saturate(desiredPitch,80,-80);
}//end altitude control 
float correct_wrap(float current_heading)
{
// Correct 360 deg wrap around
if (current_heading >= 360)
        current_heading -= 360;
else if (current_heading < 0)
        current_heading += 360;

return(current_heading);
}


// Find the shortest signed angular difference between a target and source angle
float angular_diff(float target_angle, float source_angle)
{
        // Find simple difference
        float diff = target_angle - source_angle;
        if (diff > 180)
                diff -= 360;
        else if (diff < -180)
                diff += 360;

        return(diff);
}
