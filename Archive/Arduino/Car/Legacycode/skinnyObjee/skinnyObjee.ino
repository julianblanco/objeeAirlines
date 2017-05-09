#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>
#include "IIRFilter.h"
#include "Navigation.h"
#include "ControlOutputs.h"
#define mySerial Serial1
Adafruit_GPS GPS(&mySerial);
#define pwmSerial Serial2
#define Matlab Serial3

#define GPSECHO  false
#define BNO055_SAMPLERATE_DELAY_MS (100)
 Adafruit_BNO055 bno = Adafruit_BNO055();


//macyeaton parking
//float wplat[] = {41.371671,41.371671,41.372056,41.372056,41.371657,41.371657,41.372462,41.372462,41.371642,41.371642,41.370875,41.370875};
//float wplong[]= { -72.100240,-72.100240,-72.100517,-72.100517,-72.100510,-72.100510,-72.100437,-72.100437,-72.100529,72.100529,-72.101282,-72.101282};

//football field
//float wplat[] = {41.372260,41.372260,41.372542,41.372542,41.372649,41.372649};
//float wplong[]= {-72.100240,-72.100240,-72.098794,-72.098794,-72.099228-72.099228};

//parade field

float wplat[] = {41.371926,41.371926,41.371684,41.371684,41.371934,41.371934};
float wplong[]= {-72.101997,-72.101997,-72.102142,-72.102142, -72.101619, -72.101619};




//Navigation Variables
/////////////////////////////////////////////////////////
float trueHeading=0;
float targetHeading=0;
float Gpsheading=0;
float currentEuler;
float eulerLast=0;
float headingError=0;

float GpsSpeed=0;
float GpsAltitude=0;
float GpsFixQual=0;

float distanceToTarget=0;
float precision = 1; // current distance to target (current waypoint)

int Fix=0;
int GpsSat=0;
int steer;
int waypointNumber = 0;  
int flagdist=0;


float targetLat=41.371671 ;
float targetLong=-72.100240;
float myLat=41.3;
float myLong=-72.100240;

 









/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");Serial.print(temp);Serial.println(" C"); Serial.println("");

  bno.setExtCrystalUse(true);
  startTimer(TC1, 0, TC3_IRQn, 2000); 
   Serial.println("Adafruit GPS library basic test!");
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  mySerial.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
  mySerial.println(PMTK_Q_RELEASE);
  Matlab.begin(9600);
  Matlab.println('a');
  pwmSerial.begin(115200);
  waypointing();
  trueHeading= courseToWaypoint(myLat, myLong, targetLat, targetLong);

// Steer straight initially
  pwmSerial.println(90);
}

void loop(void) 
{
// Grab Gyro data
////////////////////////////////////////////////////////////////////////////////////////////////
//grabs the gyro data from the sensor
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  currentEuler=euler.x();
//////////////////////////////////////////////////////////////////////////////////////////////////
   

//Aquire the GPS VALUES (PARSE)
//////////////////////////////////////////////////////////////////////////////////////////////////////
   //Uses ADAFRUIT Library to parse the GPS Data
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
   // Serial.print("Fix: "); Serial.print((int)GPS.fix);
   
    Fix=GPS.fix;
    
    //sets the parsed values to the variables for use by the rest of the program
    if (GPS.fix) {
      myLat=GPS.latitudeDegrees;
      myLong=GPS.longitudeDegrees;
      GpsSpeed=GPS.speed;
      Gpsheading=GPS.angle;
      GpsAltitude=GPS.altitude;
      GpsSat=GPS.satellites;
    }
////////////////////////////////////////////////////////////////////////////////////////////////////////


// IIR Filter
////////////////////////////////////////////////////////////////////////////////////////////////////

    //if no FIX ensures GPS Heading is set to true heading
    if(Fix==0 | GpsSpeed<2)
    {
    Gpsheading =trueHeading;

    }

    trueHeading=IIRFilter(Gpsheading,currentEuler,trueHeading,1);
    
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//Navigates
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // calculates the heading to the Waypoing
targetHeading = courseToWaypoint(myLat, myLong, targetLat, targetLong);
//calculates the distance to the waypoint
distanceToTarget = distanceToWaypoint(myLat, myLong, targetLat, targetLong);

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

//serial prints data
notify_user();
Matlabsend();
delay(10);
}//end loop



// Serial print to com port and xbee to matlab
////////////////////////////////////////////////////////////
void notify_user()
{
   static int x =0;
   x++;
   if(x==800)
   {

    Serial.print("/n/n/n/n/n/n/n/n/n/n/n/n/n/n/n/n");
    Serial.println("************************");
    Serial.print("Target Heading: ");  Serial.println(targetHeading,4);
    Serial.print("Current Heading: "); Serial.println(trueHeading,4);
    Serial.print("GPS Heading: ");Serial.println(Gpsheading,4);
    Serial.print("Gyro Heading: ");Serial.println(currentEuler,4);
    Serial.print("Heading error: "); Serial.println(headingError,4);
    Serial.print("Fix: ");    Serial.println(Fix);
    Serial.println("************************");
    Serial.println("************************");
    // Serial.print("currentLAT: ");   Serial.println(myLat,4);
    // Serial.print("currentLONG: "); Serial.println(myLong,4);
    // Serial.print("Target Lat: "); Serial.println(targetLat,4);
    // Serial.print("Target Long: ");  Serial.println(targetLong,4);
    Serial.print("SOG: "); Serial.println(GpsSpeed);
    Serial.print("wpnum: "); Serial.println((waypointNumber));
    Serial.print("Dis2Target: "); Serial.println(distanceToTarget,4);
    Serial.print("Steer: "); Serial.println(steer); 
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    x=0;
    }
}

 void Matlabsend()
{
    static int y =0;
      y++;
      if(y>400)
    {
    Matlab.println('a');
    Matlab.print("!");  Matlab.println(targetHeading,4);
    Matlab.print("@"); Matlab.println(trueHeading,4);
    Matlab.print("#");Matlab.println(Gpsheading,4);
    Matlab.print("$");Matlab.println(currentEuler,4);
    Matlab.print("%"); Matlab.println(headingError,4);
    Matlab.print("^");    Matlab.println(Fix);
    Matlab.print("&");   Matlab.println(myLat,4);
    Matlab.print("*"); Matlab.println(myLong,4);
    Matlab.print("("); Matlab.println(targetLat,4);
    Matlab.print(")");  Matlab.println(targetLong,4);
    Matlab.print("q"); Matlab.println(GpsSpeed);
    Matlab.print("w"); Matlab.println((waypointNumber));
    Matlab.print("e"); Matlab.println(distanceToTarget,4);
    Matlab.print("r");Matlab.println(GPS.hour);
    Matlab.print("t");Matlab.println(GPS.minute);
    Matlab.print("y");Matlab.println(GPS.seconds);
    // // Matlab.print("Satilites: "); Matlab.println(satilites); 
    // Matlab.print(" quality: "); Matlab.println((int)GPS.fixquality);  
      y=0;
    }
}

void waypointing()
{
  targetLat = wplat[waypointNumber];
  targetLong =wplong[waypointNumber];
  
  if(distanceToTarget<4)
  {
    
    waypointNumber=waypointNumber+1;
    //navigate(myLat, myLong, targetLat, targetLong, &targetHeading, &distanceToTarget);
    
  }
}

//Arduin Due Interupt Service Routine Code
/////////////////////////////////////////////////////////////////////
void TC3_Handler()
{
      
  TC_GetStatus(TC1, 0);

  char c = GPS.read();      
}

void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency)
{
        pmc_set_writeprotect(false);
        pmc_enable_periph_clk((uint32_t)irq);
        TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
        uint32_t rc = VARIANT_MCK/128/frequency; //128 because we selected TIMER_CLOCK4 above
        TC_SetRA(tc, channel, rc/2); //50% high, 50% low
        TC_SetRC(tc, channel, rc);
        TC_Start(tc, channel);
        tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
        tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
        NVIC_EnableIRQ(irq);
}






