#include <Wire.h>                                 
#include <Adafruit_Sensor.h>                      
#include <math.h>                                 // used by: GPS
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP085_U.h>
#include <utility/imumaths.h>
//#include "Adafruit_GPS.h"
#include "Communication.h"
#include "ControlOutputs.h"
#include "Navigation.h"
#include "GPS.h"
#include "Gyro.h"
#include "variables.h"
#include "IIRFilter.h"
#define GPS Serial1


#include <Adafruit_Sensor.h>
#include <math.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();


Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);




// 
////football field
//float wplat[] = {41.37217712,41.37261581,41.37252426,41.37258148};
//float wplong[]= {-72.09899139,-72.09873962,-72.09896087,-72.09924316};

////around mac
//float wplat[] = {41.371691,41.371654, 41.372069 ,41.372266 };
//float wplong[]= {-72.100191,-72.100556, -72.100487,-72.100412};

//macyeaton parking
float wplat[] = {41.371671,41.371657};
float wplong[]= { -72.100240,-72.100510};

volatile boolean l;



void setup() {
  //ISR
  pinMode(13,OUTPUT);
 startTimer(TC1, 0, TC3_IRQn, 4); 
 //TC1 channel 0, the IRQ for that channel and the desired frequency


//*******************************************************************************
    
  Serial.begin(9600);
  Serial.println("Inialize GYRO");
//*******************************************************************************
  GPS.begin(9600);
 
 
 //need adafruit library to send commands
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  //GPS.sendCommand(PGCMD_ANTENNA);
//******************************************************************************* 
  /* Initialise the sensor */
  //gyro
  if (!bno.begin())
  {    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  bno.setExtCrystalUse(true); 
//  targetHeading=read_gyro();

  sensor_t sensor;
  bmp.getSensor(&sensor);
   /* Initialise the sensor */
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
//*******************************************************************************
  // I2C start as master
  Wire.begin();
  // Get new Euler reading
//*******************************************************************************
//*******************************************************************************
   tiempo = millis();
   /*
   while (fix == 0)
  {
    Serial.print("Waiting for fix: ");
   
    delay(3000);
    counter = counter + 3;
    Serial.println(counter);
  }
  //take off routine
  while (tiempo < 10000)
  {
    drive_gyro_only(trueHeading, targetHeading);
    tiempo = millis();
  }
  */
  
  Serial.println("End Setup Routine");
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
  Serial.println("0");
  
  //check gyro and barometer
  checkSensor();
  //check NMEA calls line parser and extracts data, print "fix"
  //Serial.println(checkNMEA(&myLat,&myLong));
  //GPS run calls filter and converts degrees to decimal degrees
  gpsrun(&Gpsheading, targetLat, targetLong, &myLat, &myLong);
  //navigate calculates desired heading and distance to waypoint
  navigate(myLat, myLong, targetLat, targetLong, &targetHeading, &distanceToTarget);
  waypointing();
  //implements IIR filter and calcualtes heading error
  trueHeading= IIRFilter(Gpsheading,currentEuler, trueHeading,precision);
  generate_control(lastEuler, currentEuler, &trueHeading,Gpsheading,&headingError,targetHeading);
  //uses heading error to steer
   execute_control(headingError);
  //notify user
  notify_user();
 
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
///////////////////////////////////////////////////////////////////////////////////////////

void checkSensor()
{
  currentEuler = read_gyro();
  barometer=read_barometer();
  
}

float read_barometer()
{
  
   sensors_event_t event;
  bmp.getEvent(&event);
 
  /* Display the results (barometric pressure is measure in hPa) */
  if (event.pressure)
  {
    /* Display atmospheric pressue in hPa */
    //Serial.print("Pressure:    ");
    //Serial.print(event.pressure);
    //Serial.println(" hPa");
    
    /* Calculating altitude with reasonable accuracy requires pressure    *
     * sea level pressure for your position at the moment the data is     *
     * converted, as well as the ambient temperature in degress           *
     * celcius.  If you don't have these values, a 'generic' value of     *
     * 1013.25 hPa can be used (defined as SENSORS_PRESSURE_SEALEVELHPA   *
     * in sensors.h), but this isn't ideal and will give variable         *
     * results from one day to the next.                                  *
     *                                                                    *
     * You can usually find the current SLP value by looking at weather   *
     * websites or from environmental information centers near any major  *
     * airport.                                                           *
     *                                                                    *
     * For example, for Paris, France you can check the current mean      *
     * pressure and sea level at: http://bit.ly/16Au8ol                   */
     
    /* First we get the current temperature from the BMP085 */
    float temperature;
    bmp.getTemperature(&temperature);
   // Serial.print("Temperature: ");
    //Serial.print(temperature);
   // Serial.println(" C");

    /* Then convert the atmospheric pressure, and SLP to altitude         */
    /* Update this next line with the current SLP for better results      */
    float seaLevelPressure = 1025.0599;//SENSORS_PRESSURE_SEALEVELHPA;
    //Serial.print("Altitude:    "); 
    //Serial.print(bmp.pressureToAltitude(seaLevelPressure,event.pressure)); 
    //Serial.println(" m");
    //Serial.println("");
  }
  else
  {
    Serial.println("Sensor error");
  }
}
float read_gyro()
{
  // Get new Euler reading
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float x_angle = euler.x();
  
  return(x_angle);
}


void TC3_Handler()
{
      
  TC_GetStatus(TC1, 0);
  read_GPS_serial();
        
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


void read_GPS_serial(void )
{
  // If available, put new character into circular buffer
  while (GPS.available())
  {
    circ_buff_set(GPS.read());
  }
}

void waypointing()
{
  targetLat = wplat[waypointNumber];
  targetLong =wplong[waypointNumber];
  
  if(distanceToTarget<4)
  {
    waypointNumber++;
    navigate(myLat, myLong, targetLat, targetLong, &targetHeading, &distanceToTarget);
  }
  
}

void notify_user()
{
  
  
  
   static int x =0;
   x++;
   if(x==10)
   {
    Serial.println("************************");
    Serial.print("GPS Heading: ");
    Serial.println(Gpsheading,4);
    //trueHeading = (0.2*GyroHeading) + (0.8*Gpsheading);
    Serial.print("Current Heading: ");
    Serial.println(trueHeading,4);
    Serial.print("Heading error: "); Serial.println(headingError,4);
    Serial.print("Target Heading: ");  Serial.println(targetHeading,4);
    Serial.print("Fix: ");    Serial.println(Fix);
    Serial.println("************************");
    Serial.println("************************");
    Serial.print("currentLAT: ");   Serial.println(myLat,4);
    Serial.print("currentLONG: "); Serial.println(myLong,4);
    Serial.print("Target Lat: "); Serial.println(targetLat,4);
     Serial.print("Target Long: ");  Serial.println(targetLong,4);
   //  Serial.print("SOG: "); Serial.println(SpeedOverGround);
     Serial.print("wpnum: "); Serial.println(waypointNumber);
    Serial.print("Dis2Target: "); Serial.println(distanceToTarget,4);
       // Serial.print("Satilites: "); Serial.println(satilites);
    x=0;
   }
}
  
  
  
  

