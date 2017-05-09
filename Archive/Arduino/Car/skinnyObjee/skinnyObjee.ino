#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GNSS.h>
#include "IIRFilter.h"
#include "Navigation.h"
#include "ControlOutputs.h"
#include <SPI.h>
#include <SD.h>
#define mySerial Serial1
Adafruit_GPS GPS(&mySerial);
#define pwmSerial Serial2
#define i2c Serial3


#define GPSECHO  false
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();


inline void digitalWriteDirect(int pin, boolean val){
  if(val) g_APinDescription[pin].pPort -> PIO_SODR = g_APinDescription[pin].ulPin;
  else    g_APinDescription[pin].pPort -> PIO_CODR = g_APinDescription[pin].ulPin;
}

inline int digitalReadDirect(int pin){
  return !!(g_APinDescription[pin].pPort -> PIO_PDSR & g_APinDescription[pin].ulPin);
}

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
float wplat[] = {41.372518, 41.372518, 41.372236, 41.372477, 41.372518 };
float wplong[] = { -72.099085, -72.099085, -72.098959, -72.098802, -72.099085 };

//baseball
//float wplat[] ={41.373611,41.373611,41.373680,41.373215,41.373034,41.373504};
//float wplong[]={-72.098024,-72.098024,-72.097301,-72.097170,-72.097898,-72.098027};

//FUcked Football trianlge
//float wplat[] = {41.372601, 41.372601, 41.372364, 41.372349,41.372601 };
//float wplong[] = {-72.099213, -72.099213, -72.099236,-72.098999,-72.099213};

//FUcked Football trianlge
//float wplat[] = {41.372517, 41.372517, 41.372279, 41.372510,41.372309,41.372566, 41.372551,41.372607};
//float wplong[] = {-72.099213, -72.099213,-72.099004, -72.099171,-72.098791,-72.099193, -72.098962, -72.099229};

//Navigation Variables
/////////////////////////////////////////////////////////
float trueHeading = 0;
float targetHeading = 0;
float Gpsheading = 0;
float currentEuler;
float eulerLast = 0;
float headingError = 0;

float GpsSpeed = 0;
float GpsAltitude = 0;
float GpsFixQual = 0;

float distanceToTarget = 0;
float lastdistnace = 0;
float precision = 1;                                        // current distance to target (current waypoint)

int Fix = 0;
bool new_GPS_data = 0;
int GpsSat = 0;
int steer;
int waypointNumber = 0;
int flagdist = 0;


float targetLat = 41.371671 ;
float targetLong = -72.100240;

float myLat = 41.3;
float myLong = -72.100240;


float lasttargetLat = 41.371671 ;
float lasttargetLong = -72.100240;

float gpsCoef= .06;





/**************************************************************************/
/*
Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: "); Serial.print(temp); Serial.println(" C"); Serial.println("");

  bno.setExtCrystalUse(true);
  startTimer(TC1, 0, TC3_IRQn, 2000);
  Serial.println("Adafruit GPS library basic test!");
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  mySerial.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);   // 1 Hz update rate
  //GPS.sendCommand(PGCMD_ANTENNA);
  //mySerial.println(PMTK_Q_RELEASE);
  Matlab.begin(9600);
  Matlab.println('a');
  pwmSerial.begin(115200);
  waypointing();
  trueHeading = courseToWaypoint(myLat, myLong, targetLat, targetLong);

  // Steer straight initially
  pwmSerial.println(90);
  pinMode(13, OUTPUT);     
  pinMode(12, OUTPUT);   
}

void loop(void)
{
  digitalWriteDirect(12, HIGH);
  ////////////////////////////////////////////////////////////////////////////////////////////////
  //grabs the gyro data from the sensor
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  currentEuler = euler.x();
  //////////////////////////////////////////////////////////////////////////////////////////////////
digitalWriteDirect(12, LOW); 

  //Aquire the GPS VALUES (PARSE)
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  //Uses ADAFRUIT Library to parse the GPS Data
  if (GPS.newNMEAreceived()) {
    if (GPS.parse(GPS.lastNMEA()))                          // this also sets the newNMEAreceived() flag to false
    {
      new_GPS_data = 1;
      myLat = GPS.latitudeDegrees;
      myLong = GPS.longitudeDegrees;
      GpsSpeed = GPS.speed;
      Gpsheading = GPS.angle;
      GpsAltitude = GPS.altitude;
      GpsSat = GPS.satellites;
    }
    else
    {
      new_GPS_data = 0;
    }

    Fix = GPS.fix;
  }
  // Serial.print("Fix: "); Serial.print((int)GPS.fix);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////


  // IIR Filter
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  // Only use GPS heading data if there's a new fix and vehicle is travelling fast enough
  bool use_GPS = 0;
  if ( new_GPS_data && GpsSpeed > 2)
  {
    use_GPS = 1;
    new_GPS_data = new_GPS_data - 1;
  }
  else
  {
    use_GPS = 0;
  }

  if (waypointNumber > 1)
  {
    gpsCoef=.02;
    lasttargetLat = wplat[waypointNumber - 1];
    lasttargetLong = wplong[waypointNumber - 1];
    lastdistnace = distanceToWaypoint(myLat, myLong, lasttargetLat, lasttargetLong);
    if (lastdistnace < 8)
    {
      use_GPS = 0;
    }
  }


  // Update heading estimate with gyro and GPS data
  trueHeading = update_heading_estimate(Gpsheading, currentEuler, trueHeading, use_GPS,gpsCoef);

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
  headingError =  control_output(trueHeading, targetHeading);
  //steering uses heading errror and creates a command to send to the motors based on platform
  steer = update_motors(headingError);
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  pwmSerial.println(steer);


  notify_user();




  //serial prints data

  Matlabsend();
  delay(10);
}                                                           //end loop



// Serial print to com port and xbee to matlab
////////////////////////////////////////////////////////////
void notify_user()
{
  static int x = 0;
  x++;
  if (x > 50)
  {

    Serial.print(myLat, 6); Serial.print(',');
    Serial.print(myLong, 6); Serial.print(',');
    Serial.print(targetLat, 6); Serial.print(',');
    Serial.print(targetLong, 6); Serial.print(',');
    Serial.print(targetHeading, 4); Serial.print(',');
    Serial.print(Gpsheading, 4); Serial.print(',');
    Serial.print(currentEuler, 4); Serial.print(',');
    Serial.print(trueHeading, 4); Serial.print(',');
    Serial.print(headingError, 4); Serial.print(',');
    Serial.print(GpsSpeed); Serial.print(',');
    Serial.print(waypointNumber); Serial.print(',');
    Serial.print(distanceToTarget, 4); Serial.print(',');
    Serial.print(steer); Serial.print(',');
    // Serial.print(loopspeed);Serial.print(',');
    Serial.print(GPS.hour); Serial.print(',');
    Serial.print(GPS.minute); Serial.print(',');
    Serial.print(GPS.seconds); Serial.print(',');
    Serial.print((int)GPS.satellites); Serial.print(',');
    Serial.print((int)GPS.fixquality); Serial.print(',');
    Serial.println(GPS.altitude);

    x = 0;
  }
}


void waypointing()
{
  if (distanceToTarget < 3)
  {
    waypointNumber = waypointNumber + 1;
    //navigate(myLat, myLong, targetLat, targetLong, &targetHeading, &distanceToTarget);
    //targetLat = wplat[waypointNumber];
    //targetLong =wplong[waypointNumber];
    //distanceToTarget = distanceToWaypoint(myLat, myLong, targetLat, targetLong);
  }

  //    if(waypointNumber>numWaypoints)
  //      {

  //        waypointNumber=1;
  //      }
  targetLat = wplat[waypointNumber];
  targetLong = wplong[waypointNumber];


}

//Arduin Due Interupt Service Routine Code
/////////////////////////////////////////////////////////////////////
void TC3_Handler()
{

  TC_GetStatus(TC1, 0);
digitalWriteDirect(13, HIGH);   
  char c = GPS.read();
  char d = i2c.read();
  digitalWriteDirect(13, LOW); 
}

void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency)
{
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)irq);
  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
  uint32_t rc = VARIANT_MCK / 128 / frequency;        //128 because we selected TIMER_CLOCK4 above
  TC_SetRA(tc, channel, rc / 2);                      //50% high, 50% low
  TC_SetRC(tc, channel, rc);
  TC_Start(tc, channel);
  tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;
  NVIC_EnableIRQ(irq);
}
