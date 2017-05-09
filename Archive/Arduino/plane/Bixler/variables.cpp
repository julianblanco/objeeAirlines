//#include "variables.h"
//Football trianlge
#include <stdint.h>
float wplat[] = {41.372321,41.372321, 41.371905, 41.372399, 41.371905 };
float wplong[] = {-72.099001,-72.099001, -72.098962, -72.098908, -72.098962 };

//Variables

//Navigation Variables
/////////////////////////////////////////////////////////
//Waypoint holds 
float trueHeading=0;
float targetHeading=0;
float Gpsheading=0;
float eulerLast=0;
float headingError=0;
float courseBetweenWaypoints=0;
float distanceBetweenWaypoints=0;
float oldHeading=0;
float GpsSpeed=0;
float GpsAltitude=0;
float GpsFixQual=0;
float yawOffset;
float distanceToTarget=0;
float lastdistnace=0;
float precision = 1;                                        // current distance to target (current waypoint)

int Fix=0;
 bool new_GPS_data = 0;
int GpsSat=0;
int steer;
int waypointNumber = 0;  
int flagdist=0;
float distanceXT=0;

float targetLat=41.371671 ;
float targetLong=-72.100240;

float myLat=41.3;
float myLong=-72.100240;

float lasttargetLat=41.371671 ;
float lasttargetLong=-72.100240;
 bool use_GPS = 0;

float gpsCoef =.05;

volatile uint32_t ISR_count=0;
volatile bool l;  // Count variable
volatile uint8_t orient_update = 0;  // Update orientation flag
volatile uint8_t iir_update = 0;  // Update servo positions
volatile uint8_t control_update = 0;  // Update airspeed measurement
volatile uint8_t gps_update = 0;  // Update airspeed measurement
volatile uint8_t navigate_update = 0;  // Update airspeed measurement
volatile uint8_t User_update = 0;
volatile uint8_t servo_update = 0;  // Update servo positions


//PID

//Define Variables we'll be connecting to
float rollSetpoint, rollInput, rollServoOutput;
//Define Variables we'll be connecting to
float pitchSetpoint, pitchInput, pitchServoOutput;
//Define Variables we'll be connecting to
float yawSetpoint, yawInput, yawServoOutput;

//Specify the links and initial tuning parameters
float rollKp = 13, rollKi = .1, rollKd = .5;

//Specify the links and initial tuning parameters
float pitchKp = 13, pitchKi = .1, pitchKd = 1;

//Specify the links and initial tuning parameters
float yawKp = 10, yawKi = .1, yawKd = .5;

float rollIntergral = 0;

float pitchIntergral = 0;

float yawIntegral = 0;

float pitchResponse =0;

float yawResponse =0;
//void UpdateOrientation();
//void UpdateServos();
//void UpdateAirspeed();
//void CenterAll();
float derivativeSetpoint = 0;

float calibration=0;

// Loop variables
uint16_t dt;  // Stores time in micros for gyro integration
uint32_t start;
uint32_t finish;
int error;                    // Error code returned when MPU6050 is read
int temp;
long tic;
long toc;
float timed = 0;