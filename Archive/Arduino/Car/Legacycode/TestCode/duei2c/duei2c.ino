#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground
    
   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

#define ENABLE_MOTORS
#define LeftMotor 9
#define RightMotor 10

#define velocidad 200
#define P_COEF 3

#define NUMBER_WAYPOINTS 2   
#define WAYPOINT_DIST_TOLERANE  2   // tolerance in meters to waypoint; once within this tolerance, will advance to the next waypoint
#define GPSECHO  false

#define GPS_COEF .3

// 
// 
////football field
//float wplat[] = {41.37217712,41.37261581,41.37252426,41.37258148};
//float wplong[]= {-72.09899139,-72.09873962,-72.09896087,-72.09924316};

////around mac
//float wplat[] = {41.371691,41.371654, 41.372069 ,41.372266 };
//float wplong[]= {-72.100191,-72.100556, -72.100487,-72.100412};

//macyeaton parking
//float wplat[] = {41.371671,41.371657,41.375162};
//float wplong[]= { -72.100240,-72.100510,-72.101673};

//firstieparkinglot
float wplat[]={41.374540,41.375123,41.375163,41.374545,41.374481};
float wplong[] ={ -72.101681,-72.101878,-72.101666, -72.101458,-72.101677};

float currentLat;
float currentLong;
float currentHeading;
float destination_Lat;
float destination_Long;
float distanceToTarget;           // current distance to target (current waypoint)
float GpsHead;
float Gpsheading;
float GpsSpeed;
float GpsAltitude;
float GpsFixQual;
float headingError;
float myLat;
float myLong;
float present_Lat;
float present_Long;
float rawLat;
float rawLong;
float SOG;
float target_distance = 0;
//oat timer;
float timer2;
float tiempo = 0;
float targetLat;
float targetLong;
float targetHeading;




int fix=0;
int GpsSat;
int x = 0;
int useroverrideFlag = 0;
int counter = 0;
int waypointNumber = -1;  
int flagdist=0;



/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
   
Adafruit_BNO055 bno = Adafruit_BNO055();
#define mySerial Serial1

Adafruit_GPS GPS(&mySerial);

#define GPSECHO  true

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");
  
  bno.setExtCrystalUse(true);
 Serial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  mySerial.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!

#ifdef __arm__
  usingInterrupt = false;  //NOTE - we don't want to use interrupts on the Due
#else
  useInterrupt(true);
#endif

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
}

#ifdef __AVR__
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
#endif //#ifdef__AVR__

uint32_t timer = millis();
/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void) 
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  

  /*
  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  Serial.print("qW: ");
  Serial.print(quat.w(), 4);
  Serial.print(" qX: ");
  Serial.print(quat.y(), 4);
  Serial.print(" qY: ");
  Serial.print(quat.x(), 4);
  Serial.print(" qZ: ");
  Serial.print(quat.z(), 4);
  Serial.println("");
  */
  
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
   
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
     if(fix)
  {
    rawLat =GPS.latitude;
    rawLong=GPS.longitude;
    GpsHead=GPS.angle;
    GpsFixQual=GPS.fixquality;
    GpsSpeed=GPS.speed;
    GpsAltitude=GPS.altitude;
    GpsSat=GPS.satellites;
  }
   if(fix==1)
{
   Serial.println("************************");
    Serial.print("GPS Heading: ");
    Serial.println(GpsHead,4);
    //currentHeading = (0.2*GyroHeading) + (0.8*GpsHeading);
    Serial.print("Current Heading: ");
    Serial.println(currentHeading,2);
    Serial.print("Heading error: "); Serial.println(headingError,4);
    Serial.print("Target Heading: ");  Serial.println(targetHeading,4);
    Serial.print("Fix: ");    Serial.println(fix);
    Serial.println("************************");
    Serial.println("************************");
    Serial.print("currentLAT: ");   Serial.println(myLat,6);
    Serial.print("currentLONG: "); Serial.println(myLong,6);
    Serial.print("Target Lat: "); Serial.println(targetLat,6);
     Serial.print("Target Long: ");  Serial.println(targetLong,6);
     Serial.print("SOG: "); Serial.println(GpsSpeed);
     Serial.print("wpnum: "); Serial.println(waypointNumber);
    Serial.print("Dis2Target: "); Serial.println(distanceToTarget,4);
      Serial.print("Satilites: "); Serial.println(GpsSat);
    Serial.print("alt: ");Serial.println(GpsAltitude);
  processGPS(rawLat,rawLong);
  courseToWaypoint(myLat,myLong,targetLat,targetLong);
  distanceToTarget=distanceToWaypoint(myLat,myLong,targetLat,targetLong);
}
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  
  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print(" Z: ");
  Serial.print(euler.z());
  Serial.println("");
    }
  }
  





void processGPS(float rawLat , float rawLong)
{

  myLat = convertDegMinToDecDeg(rawLat);
  myLong = convertDegMinToDecDeg(rawLong);
  myLong=myLong*-1;
  //Serial.println(myLat, 8);
 // Serial.println(myLong, 8);

//  if (GPS.latitudeDegrees == 'S')            // make them signed
//    myLat = -1 * myLat;
//  if (GPS.longitudeDegrees = 'W')
//    myLong = -1 * myLong;


}   // processGPS(void)
// converts lat/long from Adafruit degree-minute format to decimal-degrees; requires <math.h> library
float convertDegMinToDecDeg (float degMin)
{
  double min = 0.0;
  double decDeg = 0.0;

  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);

  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );

  return decDeg;
}
float courseToWaypoint(float lat1, float long1, float lat2, float long2)
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  float dlon = radians(long2 - long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float a1 = sin(dlon) * cos(lat2);
  float a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  return degrees(a2);
}  // courseToWaypoint()
//**********************************************************************************
void nextWaypoint(void)
{
  waypointNumber++;
  targetLat = wplat[waypointNumber];
  targetLong = wplong[waypointNumber];

  if ( waypointNumber >= NUMBER_WAYPOINTS)    // last waypoint reached?
  {

    Serial.println(F("* LAST WAYPOINT *"));

  }

  processGPS(rawLat,rawLong);
  //distanceToTarget = originalDistanceToTarget = distanceToWaypoint();
  targetHeading = courseToWaypoint(myLat, myLong, targetLat, targetLong);

}  // nextWaypoint()
//**********************************************************************************
float distanceToWaypoint(float Lat1, float Long1, float Lat2, float Long2)
{
  float dist;
  float dLat = (float)(Lat2 - Lat1);                                    // difference of latitude in 1/10 000 000 degrees
  float dLon = (float)(Long2 - Long1) * cos(Lat1) ; //
  dist = sqrt(sq(dLat) + sq(dLon)) * 110575;
  float olddist;
  if (dist <= WAYPOINT_DIST_TOLERANE)
  {
    if (flagdist == 0)
    {
      flagdist = 1;
      waypointNumber++;
    }
    if (waypointNumber == NUMBER_WAYPOINTS);
    {
      waypointNumber == 100;
    }

    olddist = distanceToWaypoint(wplat[waypointNumber - 1], wplong[waypointNumber - 1], myLat, myLong);
    if ((olddist > (WAYPOINT_DIST_TOLERANE * 3)))
    {
      flagdist = 0;
    }

  }
  return dist;
}
//**********************************************************************************
float Update_GPS_Heading(float GpsHead)
{
 //insert filter here
 
  float GPS_Heading=GpsHead;
  return GPS_Heading;
}
//**********************************************************************************
float update_gyro_heading(float last_euler, float current_euler,float current_heading)
{
  // Move Euler reading from last loop
 
 

  // Update current heading given the change in the gyro angle since last polled
  current_heading = current_heading + (last_euler - current_euler); // update state estimation

  // Correct 360 deg wrap around
  current_heading = correct_wrap(current_heading);
  last_euler = current_euler;
  return (current_heading);
}
//**********************************************************************************
// Use the GPS reported true heading to anchor the gyro estimated heading to a true heading
float GPS_complement(float current_heading,float Gps_heading)
{
  // Complement GPS data with regularly updated gyro data
  current_heading = GPS_COEF * Gps_heading + (1 - GPS_COEF) * (current_heading);

  // Correct 360 deg wrap around
 current_heading = correct_wrap(current_heading);

  return (current_heading);
}
//**********************************************************************************
// Fix any 360 degree wrap around that may occur while updating angular position
float correct_wrap(float current_heading)
{

  // Correct 360 deg wrap around
  if ( current_heading > 360)
    current_heading = current_heading - 360;

  if (current_heading < 0)
    current_heading = current_heading + 360;

  return (current_heading);
}
//**********************************************************************************
/*float read_gyro(void)
{
  // Get new Euler reading
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float x_angle = euler.x();
  Serial.print(euler.x());
  return(x_angle);
}
//**********************************************************************************
*/
float read_gyro(void)
{  sensors_event_t event; 
  bno.getEvent(&event);
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
   float x_angle = event.orientation.x;
 
  return(x_angle);
}
  
