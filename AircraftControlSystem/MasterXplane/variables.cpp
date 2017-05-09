//#include "variables.h"
//Football trianlge
#include <stdint.h>
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
float heading=0;
float iirheading=0;

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



//soccer

//float wplat[] =   {41.374463,41.374463,41.375865,41.37334,41.374344,41.375208,41.373938};
//float wplong[] = {-72.098189,-72.098189,-72.097946,-72.097175,-72.098258,-72.097726,-72.097720};


//float alt[]= {       5,       5,         5,        5,        5,          5,         5 ,            5,      5 ,          5};

//footballtest
// float wplat[]={41.372161,41.372161,41.371930,41.371928,41.372502};
// float wplong[]={ -72.098909,-72.098909, -72.099127,-72.099059, -72.098999};


//Groton New London Airport
//float wplat[]={41.329747,41.330392,41.328780,41.327598,41.328960,41.328682,41.329656};
//float wplong[]={-72.043812, -72.045163,-72.047273,-72.045747,-72.046576,-72.045130,-72.045850};
 //float alt[]=      {       50,50,50,50,50,50,50,50,50,50,50,0,0,0,0,0,0,0,0,0,0};
 // float alt[]=      {       200,       550,         100,        400,        200,         100,         50 ,            200,      100 ,          5,0,0,0,0,0,0,0,0,0,0};


// float wplat[]={41.336169,41.333780,41.333047,41.327596,41.325469,41.325469,41.331608};
// float wplong[]={ -72.038062, -72.046172,-72.053006,-72.045739, -72.042853,-72.042853, -72.042180};

// float wplat[]={41.330638,41.330850,41.332188,41.332201};
// float wplong[]={-72.047864,-72.044695,-72.044816, -72.047593};


float wplat[]={41.330638,41.330734,41.331677,41.331716};
float wplong[]={-72.047864, -72.045895, -72.045995,  -72.047666};
//float wplat[]={41.402922,41.402416,41.403003,41.403449,41.401942,41.403350,41.402821};
//float wplong[]={ -71.885262,-71.884421, -71.884031,-71.885549,-71.884002, -71.882750, -71.885360};
float alt[]=      {       350,     350,         350,       350,        350,         50,         4 ,            100,      100 ,          5,0,0,0,0,0,0,0,0,0,0};



float lastlat = 0;
float lastlong =0;


float myAltitude=0;
float intialpressure=0;
float intialAltitude=0;
float pressure=0;
int waypointNumber = 0;  

//7-1 becuase 0 based
int numOfWaypoints=4;
float lasttargetLat=41.371671 ;
float lasttargetLong=-72.100240;
//Variables

//Navigation Variables
/////////////////////////////////////////////////////////
//Waypoint holds 
float trueHeading=0;
float gpsHeading=0;
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

float winddir = 0;
float windmag = 0;
float AirSpeed=0;
float desiredSpeed;
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

float myLat=12;
float myLong=-56;




float gpsCoef =.05;

float rollTargetValue;
float headingErrorOld;
//PID

//Define Variables we'll be connecting to
float rollSetpoint, rollInput, rollServoOutput=0;
float pitchInput, pitchServoOutput=0;
//Define Variables we'll be connecting to
float pitchSetpoint =14;
//Define Variables we'll be connecting to
float yawSetpoint, yawInput, yawServoOutput=0;

//Specify the links and initial tuning parameters
float rollKp = .009, rollKi = .001, rollKd = .2;

//Specify the links and initial tuning parameters
float pitchKp =.005, pitchKi = .00006, pitchKd =.35;

//Specify the links and initial tuning parameters
float headingKp = 1, headingKi = .05, headingKd = .5;  // 1,.05,.5

float rollAccum=0;
float pitchAccum=0;
float headingAccum=0;

float rollyawAccum=0;
float altitudeAccum=0;
float airspeedAccum=0;

float rollErrorOld = 0;
float pitchErrorOld = 0;
float yawErrorOld = 0;
float rollYawErrorOld = 0;
float altitudeErrorOld = 0;
float airspeedErrorOld = 0;


float pitchResponse =0;
float yawResponse =0;

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

float targetAlt=5;


//%---------------------------------------
//Variables for following robot

float masterLat=0;
float masterLong=0;
float masterAlt=0;
float masterSpeed=0;
float masterHeading=0;

float throtSetpoint=1500;
float throtAcccum=0;
float throtlast;
//float desiredthrotSetpoint=1500;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
int armed =1;

int Hour;
int Minute;
int Seconds;

float pitch_offset ;

float projectedLat;
float projectedLong;


int counter_test = 0;

float templat=42.3292;
float templong=-72.039169;
float distancetraveled;
 float lasttime = 0;
 uint32_t tiempo=0;

float kalmanLat;
float kalmanLong;

float timetravel=0;
float timetook=0;
float headingOffset=0;
int takeoff_flag=1;
uint32_t counter=0;