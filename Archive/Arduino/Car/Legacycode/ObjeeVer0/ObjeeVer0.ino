#include <Wire.h>                                 // used by: motor driver
#include <Adafruit_Sensor.h>                      // part of mag sensor
#include <math.h>                                 // used by: GPS
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();

#define mySerial Serial1
Adafruit_GPS GPS(&mySerial);

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
float timer;
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






void setup() {
  Serial.begin(9600);
  Serial.println("Adafruit GPS library basic test!");

  GPS.begin(9600);
  mySerial.begin(9600);
  Serial.println("Inialize GYRO");
//*******************************************************************************
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
//******************************************************************************* 
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  bno.setExtCrystalUse(true); 
  //targetHeading=read_gyro();
//*******************************************************************************
  // I2C start as master
  Wire.begin();
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW); 
  // Get new Euler reading
//*******************************************************************************
//*******************************************************************************
   Update_GPS_Heading(GpsHead);
   GPS_run();
   while (fix == 0)
  {
    GPS_run();
    if (timer2 > millis())  timer2 = millis();
  // approximately every 2 seconds or so, print out the current stats
    if (millis() - timer2 > 2000) { 
    counter = counter + 1;
    timer2 = millis(); // reset the timer    
    Serial.print("Waiting for fix: ");
    Serial.println(counter);
    analogWrite(LeftMotor, 0);
    analogWrite(RightMotor, 0);
    counter=0;
    }
  }
   digitalWrite(13, HIGH);
  tiempo = millis();
  while (tiempo < 10000)
  {
    drive_gyro_only(currentHeading, targetHeading);
    tiempo = millis();
  }
}//end setup
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  GPS_run();
  processGPS(rawLat,rawLong);
  courseToWaypoint(myLat,myLong,targetLat,targetLong);
  distanceToTarget=distanceToWaypoint(myLat,myLong,targetLat,targetLong);
  Update_GPS_Heading(GpsHead);
  headingError=control_output(currentHeading, targetHeading);
  update_motors(headingError);
  Update_User();
  delay(100);
}
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////



void GPS_run(void)
{
 
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();
  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
  
  fix=GPS.fix;
  if(fix)
  {
    rawLat =GPS.latitude;
    rawLong=GPS.longitude;
    GpsHead=GPS.angle;
    GpsFixQual=GPS.fixquality;
    GpsSpeed=GPS.speed;
    GpsAltitude=GPS.altitude;
    GpsSat=GPS.satellites;
  }//end if fix
//
  }
}//end GPS_run
//**********************************************************************************
float read_gyro(void)
{
  // Get new Euler reading
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float x_angle = euler.x();
  
  return(x_angle);
}
//**********************************************************************************
// Calculate control response and send control output to the motors
void update_motors(float heading_error)
{
  // Calculate left/right motor response (positive -> right)
  float response = P_COEF * heading_error;
  int PWM_left = velocidad - (int)response;
  int PWM_right = velocidad + (int)response;


  // Bound PWM values
  if (PWM_left > 255)
    PWM_left = 255;
  else if (PWM_left < 0)
    PWM_left = 0;

  if (PWM_right > 255)
    PWM_right = 255;
  else if (PWM_right < 0)
    PWM_right = 0;

 if (PWM_right < 150)
        PWM_right = 150;
    if (PWM_left < 150)
        PWM_left = 150;
  // Write responses to each motor
#ifdef ENABLE_MOTORS

  analogWrite(LeftMotor,  PWM_left );
  analogWrite(RightMotor, PWM_right);
  
#endif


}
void processGPS(float rawLat , float rawLong)
{

  myLat = convertDegMinToDecDeg(rawLat);
  myLong = convertDegMinToDecDeg(rawLong);
  Serial.println(myLat, 8);
  Serial.println(myLong, 8);

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
// Calculate the current error between the current and desired headings
float control_output(float current_heading, float target_heading)
{
  float heading_error;
  // Calculate the heading error given the target and current headings
 heading_error = target_heading - current_heading;

  if ( heading_error >= 180)
  heading_error =heading_error - 360;

  if ( heading_error < -180)
    heading_error = heading_error + 360;

  return (heading_error);
}
//**********************************************************************************
void drive_gyro_only(float current_heading, float target_heading)
{
  
  float headerror=control_output(current_heading,target_heading);
   update_motors(headerror);
   
  
}


void Update_User()
{
  if(x==50||x==100){
    
    
    Serial.println("************************");
    Serial.print("GPS Heading: ");
    Serial.println(GpsHead,4);
    //currentHeading = (0.2*GyroHeading) + (0.8*GpsHeading);
    Serial.print("Current Heading: ");
    Serial.println(currentHeading,4);
    Serial.print("Heading error: "); Serial.println(headingError,4);
    Serial.print("Target Heading: ");  Serial.println(targetHeading,4);
    Serial.print("Fix: ");    Serial.println(fix);
    Serial.print("Dis2Target: "); Serial.println(distanceToTarget,4);
    }
    if(x==150){
   
    Serial.println("************************");
    Serial.print("GPS Heading: ");
    Serial.println(GpsHead,4);
    //currentHeading = (0.2*GyroHeading) + (0.8*GpsHeading);
    Serial.print("Current Heading: ");
    Serial.println(currentHeading,4);
    Serial.print("Heading error: "); Serial.println(headingError,4);
    Serial.print("Target Heading: ");  Serial.println(targetHeading,4);
    Serial.print("Fix: ");    Serial.println(fix);
    Serial.println("************************");
    Serial.println("************************");
    Serial.print("currentLAT: ");   Serial.println(currentLat,6);
    Serial.print("currentLONG: "); Serial.println(currentLong,6);
    Serial.print("Target Lat: "); Serial.println(targetLat,6);
     Serial.print("Target Long: ");  Serial.println(targetLong,6);
     Serial.print("SOG: "); Serial.println(GpsSpeed);
     Serial.print("wpnum: "); Serial.println(waypointNumber);
    Serial.print("Dis2Target: "); Serial.println(distanceToTarget,4);
      Serial.print("Satilites: "); Serial.println(GpsSat);
    Serial.print("alt: ");Serial.println(GpsAltitude);
    
    x=0;
    }
    
}
