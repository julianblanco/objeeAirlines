//variables.h

#define mySerial Serial1


#define minAirspeed 15
#define maxAirspeed 30
#include "includes.h"

#include "Arduino.h"
//Football trianlge

extern float wplat[] ;
extern float wplong[] ;
extern float alt[] ;
//Objects
//%----------------------------
// extern Servo aileron;
// extern Servo elevator;
// extern Servo rudder;


extern Adafruit_BNO055 bno ;
extern Adafruit_BMP085 bmp;
extern float rollTargetValue;
extern float headingErrorOld;

extern GNSS gnss;
//%----------------------------


extern float iirheading;
extern float heading;

//Variables
extern float intialpressure;
extern float myAltitude;
extern float intialAltitude;

extern float pressure;
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

extern float winddir;
extern float windmag;
extern float AirSpeed;
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
extern int numOfWaypoints;
extern int flagdist;


extern float targetLat;
extern float targetLong;

extern float myLat;
extern float myLong;

extern float lasttargetLat;
extern float lasttargetLong;
extern bool use_GPS;


extern float gpsCoef;



   extern float lastlat;
   extern float lastlong ;




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
extern float headingKp ;
extern float headingKi ;
extern float headingKd ;


extern int userollKp;
extern int userollKi;
extern int userollKd;
extern int usepitchKp;
extern int usepitchKi;
extern int usepitchKd;
extern int useyawKp;
extern int useyawKi;
extern int useyawKd;

extern float rollAccum;
extern float pitchAccum;
extern float headingAccum;
extern float headingOffset;
extern float headingSetpoint;
extern float rollyawAccum;
extern float altitudeAccum;
extern float airspeedAccum;
extern float leftServoOutput;
extern float rightServoOutput;
extern float rollErrorOld;
extern float pitchErrorOld;
extern float yawErrorOld;
extern float rollYawErrorOld;
extern float altitudeErrorOld;
extern float airspeedErrorOld;

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



extern  float targetAlt;



//%---------------------------------------
//Variables for following robot

extern float masterLat;
extern float masterLong;
extern float masterAlt;
extern float masterSpeed;
extern float masterHeading;

extern float throtSetpoint;
//extern float desiredthrotSetpoint;
extern String inputString ;         // a string to hold incoming data
extern boolean stringComplete;  // whether the string is complete
extern int armed;
extern int Hour;
extern int Minute;
extern int Seconds;


extern float pitch_offset ;




extern int counter_test;