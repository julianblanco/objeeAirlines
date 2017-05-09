#include <Wire.h>                                 
#include <Adafruit_Sensor.h>                      
#include <math.h>                                 // used by: GPS
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Adafruit_GPS.h"
#include <Arduino.h>
//#include "Communication.h"
#include "ControlOutputs.h"
#include "Navigation.h"
#include "GPS.h"
#include "Gyro.h"
#include "variables.h"
#include "IIRFilter.h"
#include <Adafruit_Sensor.h>
#include <math.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <utility/imumaths.h>






#define mySerial Serial1
#define pwmSerial Serial2

Adafruit_GPS GPS(&mySerial);
 
#define GPSECHO  false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy



#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);


  



// 
////football field
//float wplat[] = {41.37217712,41.37261581,41.37252426,41.37258148};
//float wplong[]= {-72.09899139,-72.09873962,-72.09896087,-72.09924316};

////around mac
//float wplat[] = {41.371691,41.371654, 41.372069 ,41.372266 };
//float wplong[]= {-72.100191,-72.100556, -72.100487,-72.100412};

//macyeaton parking
float wplat[] = {41.371671,41.371671,41.372056,41.372056,41.371657,41.371657,41.372462,41.372462,41.371642,41.371642,41.370875,41.370875};
float wplong[]= { -72.100240,-72.100240,-72.100517,-72.100517,-72.100510,-72.100510,-72.100437,-72.100437,-72.100529,72.100529,-72.101282,-72.101282};

volatile boolean l;

uint32_t timer = millis();

void setup() {
  //ISR
  pinMode(13,OUTPUT);
 startTimer(TC1, 0, TC3_IRQn, 10); 
 //TC1 channel 0, the IRQ for that channel and the desired frequency


//*******************************************************************************
    
  Serial.begin(9600);
  Serial.println("Inialize GYRO");
//*******************************************************************************
#ifdef __arm__
  usingInterrupt = false;  //NOTE - we don't want to use interrupts on the Due
#else
  useInterrupt(true);
#endif

    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  mySerial.begin(9600);
  pwmSerial.begin(9600);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
  
 //need adafruit library to send commands
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
  Serial.println("Past GPS");
//******************************************************************************* 

//*******************************************************************************
  // I2C start as master
  Wire.begin();
  // Get new Euler reading
//*******************************************************************************
  /* Initialise the sensor */
  //gyro
  if (!bno.begin())
  {    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  bno.setExtCrystalUse(true); 
//  targetHeading=read_gyro();

  
//*******************************************************************************
 
    Serial.println("End Setup Routine");
    //   int wait=0;
    //  while(Fix == 0)
    //  {
    //   wait++;
    //   if(wait==10000)
    //   {
    // Serial.println("Waiting for fix");
    // wait=0;
    //   }
     
    // }
}//end setup
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
void loop() {
 // Serial.println("0");
  
  //check gyro and barometer
  //read_gyro();
  slow++;
  if(slow ==20)
  { 
    noInterrupts();
  sensors_event_t event; 
  bno.getEvent(&event);
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  slow=0;
  interrupts();
  }
  //check NMEA calls line parser and extracts data, print "fix"
  //Serial.println(checkNMEA(&myLat,&myLong));
  GPSCall();
  //GPS run calls filter and converts degrees to decimal degrees
  gpsrun(&Gpsheading, targetLat, targetLong, &myLat, &myLong);
  //navigate calculates desired heading and distance to waypoint
  navigate(myLat, myLong, targetLat, targetLong, &targetHeading, &distanceToTarget);
  waypointing();
  //implements IIR filter and calcualtes heading error
  trueHeading= IIRFilter(Gpsheading,currentEuler, trueHeading,precision);
  generate_control(lastEuler, currentEuler, &trueHeading,Gpsheading,&headingError,targetHeading);
  //uses heading error to steer
   int pwmspeed=execute_control(headingError);
   pwmSerial.println(pwmspeed);

  //notify user
  notify_user();
delayMicroseconds(100000);
}
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////



// void read_gyro()
// {
//   // Get new Euler reading
  
// }


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

void GPSCall()
{
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

 
    Fix=GPS.fix;
    
    if (GPS.fix) {
      myLat=GPS.latitudeDegrees;
      myLong=GPS.longitudeDegrees;
      GpsSpeed=GPS.speed;
      Gpsheading=GPS.angle;
      GpsAltitude=GPS.altitude;
      GpsSat=GPS.satellites;
    }//endif
  }//end function
  
void waypointing()
{
  targetLat = wplat[waypointNumber];
  targetLong =wplong[waypointNumber];
  
  if(distanceToTarget<4)
  {
    noInterrupts();

    waypointNumber=waypointNumber+1;
    navigate(myLat, myLong, targetLat, targetLong, &targetHeading, &distanceToTarget);
    interrupts();
  }
  
}

void notify_user()
{
   static int x =0;
   x++;
   if(x==800)
   {
    if(Fix==0)
    {
    Gpsheading =trueHeading;
    }
    trueHeading = (0.6*trueHeading) + (0.4*Gpsheading);
    Serial.println("************************");
    Serial.print("Target Heading: ");  Serial.println(targetHeading,4);
    Serial.print("Current Heading: "); Serial.println(trueHeading,4);
    Serial.print("GPS Heading: ");Serial.println(Gpsheading,4);
    Serial.print("Gyro Heading: ");Serial.println(currentEuler,4);
    Serial.print("Heading error: "); Serial.println(headingError,4);
    Serial.print("Fix: ");    Serial.println(Fix);
    Serial.println("************************");
    Serial.println("************************");
    Serial.print("currentLAT: ");   Serial.println(myLat,4);
    Serial.print("currentLONG: "); Serial.println(myLong,4);
    Serial.print("Target Lat: "); Serial.println(targetLat,4);
    Serial.print("Target Long: ");  Serial.println(targetLong,4);
    Serial.print("SOG: "); Serial.println(GpsSpeed);
    Serial.print("wpnum: "); Serial.println((waypointNumber));
    Serial.print("Dis2Target: "); Serial.println(distanceToTarget,4);
    // Serial.print("Satilites: "); Serial.println(satilites);
   x=0;
   }
}
  
  