#include <Wire.h>

#include "files/Adafruit_Sensor.h"
#include "files/Adafruit_BNO055.h"
#include "files/Adafruit_BMP085.h"
#include "files/utility/imumaths.h"
#include "files/GNSS.h"
#include "IIRFilter.h"
#include "Navigation.h"
#include "ControlOutputs.h"
#include <SPI.h>
#include <SD.h>
#define mySerial Serial1
GNSS GNSS(&mySerial);
#define pwmSerial Serial2

Adafruit_BNO055 bno = Adafruit_BNO055();

float GNSSCoef=0.03;

//Football trianlge
float wplat[] = {41.372518,41.372518, 41.372236, 41.372477, 41.372518 };
float wplong[] = {-72.099085,-72.099085, -72.098959, -72.098802, -72.099085 };

//Navigation Variables
/////////////////////////////////////////////////////////
//Waypoint holds 
float trueHeading=0;
float targetHeading=0;
float GNSSheading=0;
float currentEuler;
float eulerLast=0;
float headingError=0;

float GNSSSpeed=0;
float GNSSAltitude=0;
float GNSSFixQual=0;

float distanceToTarget=10;
float lastdistnace=10;
float precision = 1;                                        // current distance to target (current waypoint)

int Fix=0;
bool new_GNSS_data = 0;
int GNSSSat=0;
int steer;
int waypointNumber = 1;  
int flagdist=0;


float targetLat=41.371671 ;
float targetLong=-72.100240;

float myLat=41.3;
float myLong=-72.100240;

float lasttargetLat=41.371671 ;
float lasttargetLong=-72.100240;
 bool use_GNSS = 0;


float courseBetweenWaypoints=0;
float oldHeading=0;
float distanceXT=0;

uint32_t counter=0;
#define delayTime 10
/**************************************************************************/
/*
Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void) 
{
  Wire.begin();  // Initialize I2C bus
  Serial.begin(9600);
  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  bno.setExtCrystalUse(true);

  Serial.println("objeeAirlines Autonomous Veichile Log File");
  GNSS.begin(9600);
  mySerial.begin(9600);
 
 
  pwmSerial.begin(115200);
 
  waypointing(); // curious, cause of double waypoint startup?
  trueHeading= 180; // curious, initialize with magnetic? 
  pwmSerial.println(90);
}

void loop(void) 
{
  uint32_t tiempo=millis();
 
  if (counter%10)  UpdateOrientation();
  if (counter%50)  GNSS_Sample();
  if (counter%50)  IIRFilter();
  if (counter%200) Navigation();
  if (counter%10) Update_Control();
  if (counter%10000) notify_user();

  counter++;
  //delayTime =10 so should be 100 hz
  while(millis()<tiempo+delayTime){;}
}

void UpdateOrientation()
{
    // Grab Gyro data
    ////////////////////////////////////////////////////////////////////////////////////////////////
    //grabs the gyro data from the sensor
  
    // Only use GNSS heading data if there's a new fix and vehicle is travelling fast enough
       imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      currentEuler=euler.x();

    if(new_GNSS_data > 0 && GNSSSpeed > 2)
    {
        use_GNSS = 1;
        new_GNSS_data=new_GNSS_data - 1;
    }
    else
    {
        use_GNSS = 0;
    }

    if(waypointNumber >1)
    {
         lasttargetLat = wplat[waypointNumber-1];
         lasttargetLong = wplong[waypointNumber-1];
         lastdistnace = distanceToWaypoint(myLat, myLong, lasttargetLat, lasttargetLong);
         if(lastdistnace<8)
         {
             use_GNSS = 0;
         }
    }


    // Update heading estimate with gyro and GNSS data
    trueHeading=update_heading_estimate(GNSSheading,currentEuler,trueHeading,use_GNSS);
    //////////////////////////////////////////////////////////////////////////////////////////////////

}

void GNSS_Sample(void)
{
    //Aquire the GNSS VALUES (PARSE)
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    //Uses ADAFRUIT Library to parse the GNSS Data
    if (GNSS.newNMEAreceived()) {
        if (GNSS.parse(GNSS.lastNMEA()))                          // this also sets the newNMEAreceived() flag to false
        {
            new_GNSS_data = 1;
            myLat=GNSS.latitudeDegrees;
            myLong=GNSS.longitudeDegrees;
            GNSSSpeed=GNSS.speed;
            GNSSheading=GNSS.angle;
            GNSSAltitude=GNSS.altitude;
            GNSSSat=GNSS.satellites;
        }
        else
        {
            new_GNSS_data = 0;
        }

        Fix=GNSS.fix;
    }
    // Serial.print("Fix: "); Serial.print((int)GNSS.fix);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////

}

void IIRFilter(){
    // IIR Filter
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    trueHeading = update_heading_estimate(GNSSheading, currentEuler, trueHeading, use_GNSS);

}

void Navigation()
{

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //Navigates
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // calculates the heading to the Waypoing
     targetHeading = courseToWaypoint(myLat, myLong, targetLat, targetLong);
     float XTerror= crossTrackError(distanceToTarget,courseBetweenWaypoints,oldHeading);
    
    //CROSSTRACK ERROR
     if(waypointNumber>1 && XTerror > .25)
     {      
       targetHeading= crossTrackCorrection(XTerror,targetHeading );
     }
  
    waypointing();

}

void Update_Control()
{
    //Creates the response for the motors
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //heading error uses PID to compute response to correct the heading error
    headingError=  control_output(trueHeading, targetHeading);
    //steering uses heading errror and creates a command to send to the motors based on platform
    steer = update_motors(headingError);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    pwmSerial.println(steer);
  
}

  
    

                                                       //end loop



// Serial print to com port and xbee to matlab
////////////////////////////////////////////////////////////
void notify_user()
{
    
        Serial.print(myLat,6);Serial.print(',');
        Serial.print(myLong,6);Serial.print(',');
        Serial.print(targetLat,6);Serial.print(',');
        Serial.print(targetLong,6);Serial.print(',');
        Serial.print(targetHeading,4);Serial.print(',');
        Serial.print(GNSSheading,4);Serial.print(',');
        Serial.print(currentEuler,4);Serial.print(',');
        Serial.print(trueHeading,4);Serial.print(',');
        Serial.print(headingError,4);Serial.print(',');
        Serial.print(GNSSSpeed);Serial.print(',');
        Serial.print(waypointNumber);Serial.print(',');
        Serial.print(distanceToTarget,4);Serial.print(',');
        Serial.print(steer);Serial.print(',');
       // Serial.print(loopspeed);Serial.print(',');
        Serial.print(GNSS.hour);Serial.print(',');
        Serial.print(GNSS.minute);Serial.print(',');
        Serial.println(GNSS.seconds);
    
}


void waypointing()
{
    if(distanceToTarget<4.5)
        {
        waypointNumber=waypointNumber+1;
        }

    targetLat = wplat[waypointNumber];
    targetLong = wplong[waypointNumber];

    
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
  float xtCoeff = -20;
  float temp = xtCoeff*distanceXT;

  if(temp>30) temp=30;
  if(temp<-30) temp=-30;

  float newTargetHeading = targetHead+temp  ;

  if(newTargetHeading >= 360) newTargetHeading -= 360;
  else if(newTargetHeading < 0) newTargetHeading += 360;

  return newTargetHeading;
} // end crossTrackError
