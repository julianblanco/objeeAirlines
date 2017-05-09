//#include "variables.h"
//Football trianlge
#include <stdint.h>

#include "Arduino.h"
float heading=0;

//float wplat[] =   {41.372321, 41.372321, 41.371905,  41.372399,  41.371905 ,41.372321, 41.371905, 41.372399, 41.371905 };
//float wplong[] = {-72.099001,-72.099001, -72.098962, -72.098908, -72.098962 -72.099001, -72.098962, -72.098908, -72.098962};

//football field
//float wplat[] =   {41.372250,   41.372250,  41.372015,41.371856,   41.372405,41.372288,41.372015,41.371856,   41.372405,41.372288};
//float wplong[] = { -72.099017, -72.099017, -72.099218,-72.099068, -72.098813, -72.099130, -72.099218,-72.099068, -72.098813, -72.099130};

//Pats Football
//float wplat[] =   {41.372296,   41.372296,  41.372012, 41.372240,  41.372417, 41.371844, 41.372554};
//float wplong[] = {  -72.099007,-72.099007, -72.099194,-72.098927, -72.099065,-72.099076,-72.098995};

//float wplat[]={41.372499,41.372499,41.372173,41.372006,41.372332};
//float wplong[]={-72.098994,-72.098994,-72.099027,-72.099046,-72.099006};


//float wplat[]= { 41.402567, 41.402567, 41.403074, 41.402953, 41.402544, 41.402701, 41.403177, 41.402953, 41.402465, 41.402771 ,0,0,0,0,0,0,0,0,0,0};
//float wplong[]={-71.884084,-71.884084,-71.884387,-71.884845,-71.884668,-71.883812,-71.884025,-71.884800,-71.884608,-71.884044 ,0,0,0,0,0,0,0,0,0,0};

//baseball
//float wplat[] =   {41.374384,   41.374384,  41.374304, 41.373842, 41.373293,  41.374384,  41.374304,  41.373842,41.373293 };
//float wplong[] = { -72.098126,  -72.098126, -72.097417,-72.097854,-72.097881, -72.098126, -72.097417,-72.097854,-72.097881};
float alt[]=      {       5,       5,         15,        10,        3,          5,         5 ,            5,      5 ,          5,0,0,0,0,0,0,0,0,0,0};

//soccer

//float wplat[] =   {41.374463,41.374463,41.375865,41.37334,41.374344,41.375208,41.373938};
//float wplong[] = {-72.098189,-72.098189,-72.097946,-72.097175,-72.098258,-72.097726,-72.097720};


//float alt[]= {       5,       5,         5,        5,        5,          5,         5 ,            5,      5 ,          5};

//footballtest
float wplat[]={41.372161,41.372161,41.371930,41.371928,41.372502};
float wplong[]={ -72.098909,-72.098909, -72.099127,-72.099059, -72.098999};

float myAltitude=0;
float intialpressure=0;
float intialAltitude=0;
float pressure=0;
int waypointNumber = 0;  
int numOfWaypoints=10;
float lasttargetLat=41.371671 ;
float lasttargetLong=-72.100240;
//Variables

//Navigation Variables
/////////////////////////////////////////////////////////
//Waypoint holds 
float trueHeading=0;
float targetHeading=0;
float Gpsheading=0;
float headingError=0;
float oldHeading=0;

float eulerLast=0;
float yawOffset=0;

float courseBetweenWaypoints=0;
float distanceBetweenWaypoints=0;
float distanceToTarget=0;
float lastdistnace=0;

float GpsSpeed=0;
float GpsAltitude=0;
float GpsFixQual=0;
int GpsSat=0;

int Fix=0;
bool new_GPS_data = 0;

int steer;
int flagdist=0;
float distanceXT=0;

float targetLat=41.371671 ;
float targetLong=-72.100240;

float myLat=41.3;
float myLong=-72.100240;


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
float rollSetpoint, rollInput, rollServoOutput=0;
float pitchInput, pitchServoOutput=0;
//Define Variables we'll be connecting to
float pitchSetpoint =14;
//Define Variables we'll be connecting to
float yawSetpoint, yawInput, yawServoOutput=0;

//Specify the links and initial tuning parameters
float rollKp = 15, rollKi = .1, rollKd = .5;

//Specify the links and initial tuning parameters
float pitchKp = 9, pitchKi = .1, pitchKd = 1;

//Specify the links and initial tuning parameters
float yawKp = 8, yawKi = .1, yawKd = .5;

float rollIntergral = 0;float pitchIntergral = 0;float yawIntegral = 0;

float pitchResponse =0;float yawResponse =0;

float derivativeSetpoint = 0;

int userollKp;
int userollKi;
int userollKd;
int usepitchKp;
int usepitchKi;
int usepitchKd;
int useyawKp;
int useyawKi;
int useyawKd;
float calibration=0;

// Loop variables
uint16_t dt;  // Stores time in micros for gyro integration
uint32_t start;
uint32_t finish;
int error;                    // Error code returned when MPU6050 is read
int temp;

float throttleOutput;
float targetAlt=5;


//%---------------------------------------
//Variables for following robot

float masterLat=0;
float masterLong=0;
float masterAlt=0;
float masterSpeed=0;
float masterHeading=0;

float throtSetpoint=1500;
//float desiredthrotSetpoint=1500;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
int armed =0;