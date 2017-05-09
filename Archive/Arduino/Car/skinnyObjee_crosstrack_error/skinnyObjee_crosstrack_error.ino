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


#define GNSSECHO  false
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();

//int numWaypoints = ;
//macyeaton parking
//float wplat[] = {41.371671,41.371671,41.372056,41.372056,41.371657,41.371657,41.372462,41.372462,41.371642,41.371642,41.370875,41.370875};
//float wplong[]= { -72.100240,-72.100240,-72.100517,-72.100517,-72.100510,-72.100510,-72.100437,-72.100437,-72.100529,72.100529,-72.101282,-72.101282};

//football field
//float wplat[] = {41.37216,41.37251,41.372564,41.37216};
//float wplong[]= {-72.09903,-72.09876,-72.09915,-72.09903};

//football field
//float wplat[] = {41.372158,41.372158,41.372146,41.372394,41.372529,41.372018,41.372664,41.372568};
//float wplong[]= { -72.099016,-72.099016,-72.098811,-72.099251, -72.098714,-72.099025, -72.098955,-72.099287};

//parade field

//float wplat[] = {41.371926,41.371926,41.371684,41.371684,41.371934,41.371934};
//float wplong[]= {-72.101997,-72.101997,-72.102142,-72.102142, -72.101619, -72.101619};

//parade field general test
//float wplat[] = {41.371787,41.372089,41.372030,41.371787};
//float wplong[]= {-72.101795,-72.101731,-72.102074,-72.101795};

//crazy turns
//float wplat[] = {41.371622,41.371717,41.371678,41.371845,41.371732};
//float wplong[]= {-72.101907,-72.101792,-72.101926,-72.101953,-72.101789};

//box
//float wplat[] = {41.371732,41.372005,41.372001};
//float wplong[]= {-72.101789,-72.101749,-72.102063};

//Around Mac
//float wplat[] = {41.371928,41.371928,41.371673,41.371689,41.371643,41.371512,41.371402,41.370836,41.370667,41.370604,41.370623,41.370510,41.370271,41.370569,41.370846,41.371262,41.371628,41.372637,41.372693,41.372913,41.372974,41.372733,41.372248};
//float wplong[]= {-72.100574,-72.100574,-72.100548, -72.100136,-72.100535, -72.100614, -72.100806,-72.101332,-72.101002,-72.100780,-72.100613, -72.100323, -72.099892,-72.099678,-72.099611,-72.099735,-72.099688,-72.099574,-72.099840,-72.099946,-72.100166,-72.100411,-72.100438};

//Soccer Field
//float wplat[] = {41.375567,41.375164,41.375269,41.375527,41.375613,41.375069};
//float wplong[] = {-72.097816,-72.097336,-72.097759,-72.098202,-72.097436,-72.098100};

//Football square
//float wplat[] ={41.372504,41.372504,41.372493,41.372482,41.372474,41.372391,41.372310,41.372231,41.372241,41.372260,41.372335,41.372500,41.372494,41.372408,41.372281,41.372160,41.372174,41.372300,41.372665,41.372504,41.372493,41.372482,41.372474,41.372391,41.372310,41.372231,41.372241,41.372260,41.372335,41.372500,41.372494,41.372408,41.372281,41.372160,41.372174,41.372300,41.372665};
//float wplong[]={-72.099176,-72.099176,-72.099040,-72.098897,-72.098773,-72.098781, -72.098792, -72.098797,-72.099006, -72.099207,-72.099200, -72.099178,-72.098974,-72.098985, -72.099001,-72.099016,-72.099217,-72.099201, -72.099160,-72.099176,-72.099040,-72.098897,-72.098773,-72.098781, -72.098792, -72.098797,-72.099006, -72.099207,-72.099200, -72.099178,-72.098974,-72.098985, -72.099001,-72.099016,-72.099217,-72.099201, -72.099160};

//Football trianlge
//float wplat[] = {41.372518,41.372518, 41.372236, 41.372477, 41.372518, 41.372236, 41.372477, 41.372518, 41.372236, 41.372477, 41.372518 };
//float wplong[] = {-72.099085,-72.099085, -72.098959, -72.098802, -72.099085 , -72.098959, -72.098802,-72.099085, -72.098959, -72.098802};

//Football straight line test
//float wplat[] = {41.372488, 41.372181};
//float wplong[] = {-72.098967, -72.099017};

//float wplat[] = {41.372517, 41.372517, 41.372279, 41.372510,41.372309,41.372566, 41.372551,41.372607};
//float wplong[] = {-72.099213, -72.099213,-72.099004, -72.099171,-72.098791,-72.099193, -72.098962, -72.099229};

//Cross Track Trial
float wplat[] =  { 41.372309, 41.372309, 41.372146, 41.372175, 41.372543};
float wplong[] = { -72.099610,-72.098791,-72.098811,-72.099229,-72.098974};

//float wplat[] =  { 41.372243, 41.371920, 41.371495, 41.372175, 41.371457 ,41.371821,  41.372148, 41.371625, 41.371434, 41.371509,  41.372432};
//float wplong[] = {-72.098791, -72.099650,-72.099713,-72.099229,-72.098897,-72.098977,-72.098944,-72.098941,-72.098895,-72.099703, -72.099602};
//Navigation Variables
/////////////////////////////////////////////////////////
float trueHeading=0;
float targetHeading=0;
float courseBetweenWaypoints=0;
float GNSSheading=0;
float currentEuler;
float eulerLast=0;
float headingError=0;

float GNSSSpeed=0;
float GNSSAltitude=0;
float GNSSFixQual=0;

float distanceBetweenWaypoints=0;
float distanceToTarget=0;
float lastdistnace=0;
float precision = 1;                                        // current distance to target (current waypoint)

int Fix=0;
bool new_GNSS_data = 0;
int GNSSSat=0;
int steer;
int waypointNumber = 0;  
int flagdist=0;


float targetLat=41.371671 ;
float targetLong=-72.100240;

float myLat=41.3;
float myLong=-72.100240;
float distanceXT;

float lasttargetLat=41.371671 ;
float lasttargetLong=-72.100240;
float oldHeading;






/**************************************************************************/
/*
Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void) 
{

  Serial.begin(9600);
  Wire.begin();
  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  startTimer(TC1, 0, TC3_IRQn, 2000); 
  Serial.println("objeeAirlines Autonoumos Ground Vehicle Log File");
  // 9600 NMEA is the default baud rate for Adafruit MTK GNSS's- some use 4800
  GNSS.begin(9600);
  mySerial.begin(9600);
  pwmSerial.begin(115200);
  waypointing();
  trueHeading= courseToWaypoint(myLat, myLong, targetLat, targetLong);
// Steer straight initially
  pwmSerial.println(90);
  delay(300);
  pwmSerial.println(110);
  delay(300);
 pwmSerial.println(70);
  delay(300);
 pwmSerial.println(90);
  delay(300);
}
void loop(void) 
{
    // Grab Gyro data
    ////////////////////////////////////////////////////////////////////////////////////////////////
    //grabs the gyro data from the sensor
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  currentEuler=euler.x();
    //////////////////////////////////////////////////////////////////////////////////////////////////


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


    // IIR Filter
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    // Only use GNSS heading data if there's a new fix and vehicle is travelling fast enough
      bool use_GNSS = 0;
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

    //calculates the distance to the waypoint
     distanceToTarget = distanceToWaypoint(myLat, myLong, targetLat, targetLong);

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
  






    //checks to see if we are close enough to the waypoint and 
     waypointing();
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




    //Creates the response for the motors
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //heading error uses PID to compute response to correct the heading error
     headingError=  control_output(trueHeading, targetHeading);
    //steering uses heading errror and creates a command to send to the motors based on platform
     steer = update_motors(headingError);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
     pwmSerial.println(steer);

     if(Fix)
     {
      notify_user();
    }
    


    //serial prints data
    
    
    delay(10);
}                                                           //end loop



// Serial print to com port and xbee to matlab
////////////////////////////////////////////////////////////
void notify_user()
{
  static int x =0;
  x++;
  if(x>50)
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
   Serial.print(oldHeading);Serial.print(',');
   Serial.println(distanceXT);
   x=0;
 }
}


void waypointing()
{
  if(distanceToTarget<2.5)
  {
    waypointNumber=waypointNumber+1;
  }


  targetLat = wplat[waypointNumber];
  targetLong = wplong[waypointNumber];
  distanceBetweenWaypoints=distanceToWaypoint(targetLat,targetLong,wplat[waypointNumber-1],wplong[waypointNumber-1]);
  courseBetweenWaypoints = courseToWaypoint(wplat[waypointNumber-1],wplong[waypointNumber-1],targetLat,targetLong);

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

//Arduin Due Interupt Service Routine Code
/////////////////////////////////////////////////////////////////////
void TC3_Handler()
{

  TC_GetStatus(TC1, 0);

  char c = GNSS.read();      
}

void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency)
{
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)irq);
  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
    uint32_t rc = VARIANT_MCK/128/frequency;            //128 because we selected TIMER_CLOCK4 above
    TC_SetRA(tc, channel, rc/2);                        //50% high, 50% low
    TC_SetRC(tc, channel, rc);
    TC_Start(tc, channel);
    tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
    tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
    NVIC_EnableIRQ(irq);
  }
