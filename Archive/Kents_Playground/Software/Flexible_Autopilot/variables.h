//variables.h


//Football trianlge
extern float wplat[] ;
extern float wplong[] ;
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>
#include "IIRFilter.h"
#include "Navigation.h"
#include "craft.h"
#include <SPI.h>
#include <SD.h>
#include "ServoM.h"
#include "ServoTimerM.h"


//Variables

//Navigation Variables
/////////////////////////////////////////////////////////
//Waypoextern int holds 
extern float trueHeading;
extern float targetHeading;
extern float Gpsheading;
extern float eulerLast;
extern float headingError;
extern float courseBetweenWaypoints;
extern float distanceBetweenWaypoints;
extern float distanceXT;
extern float oldHeading;
extern float yawOffset;
extern float GpsSpeed;
extern float GpsAltitude;
extern float GpsFixQual;

extern float distanceToTarget;
extern float lastdistnace;
extern float precision;                                        // current distance to target (current waypoextern int)

extern int Fix;
extern bool new_GPS_data;
extern int GpsSat;
extern int steer;
extern int waypointNumber; 
extern int flagdist;


extern float targetLat;
extern float targetLong;

extern float myLat;
extern float myLong;

extern float lasttargetLat;
extern float lasttargetLong;
extern bool use_GPS;


extern float gpsCoef;





//PID

//Define Variables we'll be connecting to
extern float rollSetpoint;
extern float rollInput;
extern float rollServoOutput;
//Define Variables we'll be connecting to
extern float pitchSetpoint;
extern float pitchInput;
extern float pitchServoOutput;
//Define Variables we'll be connecting to
extern float yawSetpoint;
extern float yawInput;
extern float yawServoOutput;
extern float calibration;
//Specify the links and initial tuning parameters
extern float rollKp ;
extern float rollKi ;
extern float rollKd ;

//Specify the links and initial tuning parameters
extern float pitchKp;
extern float pitchKi;
extern float pitchKd;

//Specify the links and initial tuning parameters
extern float yawKp ;
extern float yawKi ;
extern float yawKd ;

extern float rollIntergral ;

extern float pitchIntergral;

extern float yawIntegral ;

extern float pitchResponse;

extern float yawResponse ;
//void UpdateOrientation();
//void UpdateServos();
//void UpdateAirspeed();
//void CenterAll();
extern float derivativeSetpoint ;

extern uint16_t dt;  // Stores time in micros for gyro integration
extern uint32_t start;
extern uint32_t finish;
extern int error;                    // Error code returned when MPU6050 is read
extern int temp;

extern float throttleOutput;