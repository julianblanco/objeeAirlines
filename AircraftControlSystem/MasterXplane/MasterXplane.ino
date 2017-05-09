#include "wiring_private.h" // pinPeripheral() function
#define PIN 13

// Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
// void SERCOM1_Handler()
// {
//   Serial2.IrqHandler();
// }


float utmeast=0;
float utmnorth=0;
String zonez ="";

float calclat;
float calclong;

#include <SD.h>
#include "variables.h"

// #define mySerial Serial
// GNSS GNSS(&mySerial);

// Servo aileron;
// Servo elevator;
// Servo rudder;
// Servo Throttle;
// Declare the gosh darn functions
void SDinit();

int ledstate =0;
//#define delayTime 33
#define delayTime 15

void setup(void) 
{
  pinMode(20,OUTPUT);
  //servoBegin(aileron,elevator,rudder,Throttle);
  //ServoWiggle(aileron,elevator,rudder);
  //pinMode(PIN, OUTPUT);
  //centering servos
  delay(1000);
  Serial.begin(115200);
  // GNSS.begin(9600);
  //mySerial.begin(9600);
  // Assign pins 10 & 11 SERCOM functionality
  
  // XBEE.begin(9600);
  Serial2.begin(115200);
  Serial3.begin(9600);
  // pinPeripheral(10, PIO_SERCOM);
  // pinPeripheral(11, PIO_SERCOM);
  Serial.println("ObjeeAirlines Test");

  for(int idx =0; idx <1000;idx++)
  {
    recvWithStartEndMarkers();
    recvWithStartEndMarkers2();
    delay(1);
  }
  // SDinit();
  //numOfWaypoints=(int) (sizeof(wplat)/sizeof(float));
  desiredSpeed =40;
  headingOffset = trueHeading;
  takeoff();

  pitchKp =.005;
  rollKp = .009;
  userollKi=1;
  userollKd=1;
  usepitchKi=1;
  usepitchKd=1;
  waypointing(); // curious, cause of double waypoint startup?


}//end setup






void loop(void){
   tiempo=millis();
   controlTactics();
   UpdateOrientation();
   
   if ((counter%4)==0) Navigation();
   timetook = tiempo-timetravel;
   if ((counter%1)==0) notify_user();//----------
   if ((counter%5)==0) correctgyroheading();
      if ((counter%15)==0)
      { 
        notify_swarm();
       
      }


  //  if ((counter%3)==0) mavlink_pack_and_send();

   
     // latLongToUtm(myLat,myLong, utmeast, utmnorth, zonez);
     // utmToLatLong(utmeast,utmnorth, "18T" ,calclat,calclong);
 
   recvWithStartEndMarkers();
   recvWithStartEndMarkers2();
   // if((counter%50)==0) sdlog();//----------
   counter++;
    digitalWrite(20,1);
    while(millis()<(tiempo+delayTime)) {

       recvWithStartEndMarkers();
       // communication_receive();
       recvWithStartEndMarkers2();
    }
     digitalWrite(20,0);
        // if (ledstate==1)
        // {
        //   ledstate=0;
        // }
        // else
        //   {ledstate=1;}
    timetravel=tiempo;
    // Gpsheading = courseToWaypoint(lastlat,lastlong,myLat,myLong);
    //   if ((counter%2000)==0)lastlat = myLat;
    //   if ((counter%2000)==0)lastlong=myLong;
}//end main


#define sys_id 1



File myFile;
File myFile1;
char nameoffile[16];
char nameoffile1[16];
void sdlog()
{
       myFile = SD.open(nameoffile, FILE_WRITE);
       if (myFile) {
       //       #ifdef _VARIANT_ARDUINO_ZERO_
       // *     setPin = PinMASK;
       // *     clrPin = PinMASK;
       //       #else
    
        //  #endif
        myFile.print(myLat,6);myFile.print(',');
        myFile.print(myLong,6);myFile.print(',');
        myFile.print(targetLat,6);myFile.print(',');
        myFile.print(targetLong,6);myFile.print(',');
        myFile.print(targetHeading,2);myFile.print(',');
        myFile.print(Gpsheading,2);myFile.print(',');
        myFile.print(trueHeading,2);myFile.print(',');
        myFile.print(iirheading,2);myFile.print(',');
        myFile.print(headingError,2);myFile.print(',');
        myFile.print(distancetraveled,2);myFile.print(',');
        myFile.print(waypointNumber,1);myFile.print(',');
        myFile.print(distancetraveled,4);myFile.print(',');
        myFile.print(yawInput,2);myFile.print(',');
        myFile.print(rollInput,2);myFile.print(',');
        myFile.print(pitchInput,2);myFile.print(',');
        myFile.print(calibration,1);myFile.print(',');
        myFile.print(yawServoOutput,1);myFile.print(',');
        myFile.print(rollServoOutput,1);myFile.print(',');
        myFile.print(pitchServoOutput,1);myFile.print(',');
        myFile.print(yawSetpoint,2);myFile.print(',');
        myFile.print(rollSetpoint,2);myFile.print(',');
        myFile.print(pitchSetpoint,2);myFile.print(',');
        myFile.print(throtSetpoint,2);myFile.print(',');
        myFile.print(myAltitude,2);myFile.print(',');
        myFile.print(targetAlt,1);myFile.print(',');
        myFile.print(pressure,1);myFile.print(',');//myFile.print(',');
        myFile.print(distanceXT,1);myFile.print(',');
        myFile.print(Minute,1);myFile.print(',');
        myFile.print(Seconds,1);myFile.print(',');
        myFile.print(rollyawAccum,1);myFile.print(',');
        myFile.print(altitudeAccum,1);myFile.print(',');
        myFile.print(airspeedAccum,1);myFile.print(',');
        myFile.print(rollAccum,1);myFile.print(',');
        myFile.print(headingAccum,1);myFile.print(',');
        myFile.print(pitchAccum,1);myFile.print(',');
        myFile.println(tiempo,1);
        //Add gps time
        //myFile.println(controlTest,1);//25th
        //myFile.println(controlTest,1);//25th
       
       // myFile.print(loopspeed);myFile.print(',');
      }
      myFile.close();
}

void SDinit()
{
  if (!SD.begin(4)) {
   // xbee.println("initialization failed!");
    return;
  }
  //xbee.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
   // make it long enough to hold your longest file name, plus a null terminator
  int n = 0;
  snprintf(nameoffile, sizeof(nameoffile), "data%03d.txt", n); // includes a three-digit sequence number in the file name
  while(SD.exists(nameoffile)) {
    n++;
    snprintf(nameoffile, sizeof(nameoffile), "data%03d.txt", n);
  }
  Serial.println(n);
  Serial.println(nameoffile);
  //now nameoffile[] contains the name of a file that doesn't exist
   myFile= SD.open(nameoffile,FILE_READ);
   myFile.close();
   myFile= SD.open(nameoffile,FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {
    //Serial2.print("objeeAirlines Autonomous Veichile Test");
    myFile.println("Xplanep Test");
    // close the file:
    myFile.close();
    //Serial2.println("done.");
  } else {
    // if the file didn't open, print an error:
    //Serial2.println("error opening test.txt");
  }
}

int kp,ki,kd,usekp,useki,usekd;
 int controlchoice=0;
  
// Serial print to com port and xbee to matlab
////////////////////////////////////////////////////////////
void notify_user( )
{ 
         Serial.print(myLat,6); Serial.print(',');
         Serial.print(myLong,6); Serial.print(',');
         Serial.print(kalmanLat,6); Serial.print(',');
         Serial.print(kalmanLong,6); Serial.print(',');
         Serial.print(targetHeading,1); Serial.print(',');
         Serial.print(trueHeading,1); Serial.print(',');
         Serial.print(gpsHeading,1); Serial.print(',');
         Serial.print(distancetraveled,4); Serial.print(',');
         Serial.print(AirSpeed,1); Serial.print(',');
         Serial.print(waypointNumber,1); Serial.print(',');
         Serial.print(distanceToTarget,1); Serial.print(',');
         Serial.print(myAltitude,6); Serial.print(',');
         Serial.print(rollInput,1); Serial.print(',');
         Serial.print(pitchInput,1); Serial.print(',');
         Serial.print(armed,1); Serial.print(',');
         Serial.print(rollServoOutput); Serial.print(',');
         Serial.print(pitchServoOutput); Serial.print(',');
         Serial.print(rollSetpoint); Serial.print(',');
         Serial.print(pitchSetpoint); Serial.print(',');
         Serial.print(winddir); Serial.print(',');
         Serial.print(timetook); Serial.print(',');
         Serial.print(throtSetpoint); Serial.print(',');
         Serial.print(targetAlt,6); Serial.print(',');
         Serial.print(masterLat,6); Serial.print(',');
         Serial.print(masterLong,6); Serial.print(',');
         Serial.print(masterAlt,1); Serial.print(',');
         Serial.print(masterSpeed,1); Serial.print(',');
         Serial.print(projectedLat,1); Serial.print(',');
         Serial.println(projectedLat,6);
        
}

void notify_swarm( )
{ 
         Serial3.print("<02,");
         Serial3.print(myLat,6); Serial3.print(',');
         Serial3.print(myLong,6); Serial3.print(',');
         Serial3.print(myAltitude,1); Serial3.print(',');
         Serial3.print(GpsSpeed,1); Serial3.print(',');
         Serial3.print(trueHeading,1); //Serial3.print(',');
         Serial3.println('>');
}



void recvWithStartEndMarkers( ) {
    static boolean recvInProgress = false;
    
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
    const byte numChars = 74;
    static char receivedChars[numChars];             // IF BROKEN LOOK HERE, array of chars -> string
  //x
 // if (Matlab.available() > 0) {
    if ( Serial2.available() ) {
        rc =  Serial2.read();
        // Serial.print(rc);
        // Serial.println("shtisandgiggle");
        if (recvInProgress == true) {
            if (rc != endMarker) {
             //  Serial.println(ndx);
                receivedChars[ndx] = rc;
                // Serial.print(receivedChars[ndx]);
                 //  Serial.println("test");
                 //  Serial.println(receivedChars);
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                    // ndx=0;
                    // rc=NULL;
                    // memset(receivedChars, 0, sizeof(receivedChars));
                    //  Serial.println("test1");

                    //  Serial.println(receivedChars);

                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                // Serial.println("");
                // Serial.println(receivedChars);
                tokenCreator(receivedChars,sizeof(inputString));
                                    // Serial.println("test2");
                 
                stringComplete = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
                               // Serial.println("test3");

           //  Serial.println(receivedChars);

        }
    }
}


void tokenCreator(char instr[],int strleng ){
 char * pch;
  //Serial.println("Splitting string \"%s\" into tokens:\n");

// strtok returns a pointer to a character array, not an index.
// char *s = strtok(receivedBytes,",");
// Serial.println(s);
 // Serial.println(instr);
  pch = strtok (instr,",");
  int index=0;
  while (pch != NULL)
  {

    stringparse(pch,index);
    pch = strtok (NULL, ",");
    //Serial.println(a);
  index++;
  
  }
}


void stringparse(char buffer[80],int ind)
{
  //  Serial.println(buffer);
  static int choice=0;
  //Serial.print(buffer);
  if(ind==0){
  if((strcmp(buffer,"08")==0))choice=1;
  if((strcmp(buffer,"09")==0))choice=3;
  if((strcmp(buffer,"10")==0))choice=2;
  }

  if(choice==1){

    if(ind==1) myLat=atof(buffer);
    if(ind==2) myLong=atof(buffer);
    // if(ind==3) myAltitude=atof(buffer);
    if(ind==3) GpsSpeed=atof(buffer);
    if(ind==4) myAltitude=atof(buffer);
    if(ind==5) choice=0;
    projectedLat=myLat;
    projectedLong=myLong;
    templat=myLat;
    templong=myLong;
    inputString="";
    new_GPS_data=1;
  }


    if(choice==2){

    if(ind==1) pitchInput=atof(buffer);
    if(ind==2) rollInput=atof(buffer);
    if(ind==3) gpsHeading=atof(buffer);
    if(ind==4) choice=0;
    inputString="";
  }

 if(choice==3){
    if(ind==1) myAltitude=atof(buffer);
    if(ind==2) choice=0;
    inputString="";
  }}

void recvWithStartEndMarkers2( ) {
    static boolean recvInProgress = false;
    
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
    const byte numChars = 74;
    static char receivedChars[numChars];             // IF BROKEN LOOK HERE, array of chars -> string
  //x
 // if (Matlab.available() > 0) {
    if ( Serial3.available() ) {
        rc =  Serial3.read();
        // Serial.print(rc);
        // Serial.println("shtisandgiggle");
        if (recvInProgress == true) {
            if (rc != endMarker) {
             //  Serial.println(ndx);
                receivedChars[ndx] = rc;
                // Serial.print(receivedChars[ndx]);
                 //  Serial.println("test");
                 //  Serial.println(receivedChars);
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                    // ndx=0;
                    // rc=NULL;
                    // memset(receivedChars, 0, sizeof(receivedChars));
                    //  Serial.println("test1");

                    //  Serial.println(receivedChars);

                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                // Serial.println("");
                // Serial.println(receivedChars);
                tokenCreator2(receivedChars,sizeof(inputString));
                                    // Serial.println("test2");
                 
                stringComplete = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
                               // Serial.println("test3");

           //  Serial.println(receivedChars);

        }
    }
}


void tokenCreator2(char instr[],int strleng ){
 char * pch;
  //Serial.println("Splitting string \"%s\" into tokens:\n");

// strtok returns a pointer to a character array, not an index.
// char *s = strtok(receivedBytes,",");
// Serial.println(s);
 // Serial.println(instr);
  pch = strtok (instr,",");
  int index=0;
  while (pch != NULL)
  {

    stringparse2(pch,index);
    pch = strtok (NULL, ",");
    //Serial.println(a);
  index++;
  
  }
}


void stringparse2(char buffer[80],int ind)
{
  //  Serial.println(buffer);
  static int choice=0;
  //Serial.print(buffer);
  if(ind==0){
  if((strcmp(buffer,"01")==0))choice=1;
  if((strcmp(buffer,"02")==0))choice=3;
  if((strcmp(buffer,"03")==0))choice=2;
  if((strcmp(buffer,"04")==0))choice=4;
  }

  if(choice==1){

    if(ind==1) masterLat=atof(buffer);
    if(ind==2) masterLong=atof(buffer);
    // if(ind==3) myAltitude=atof(buffer);
    if(ind==3) masterAlt=atof(buffer);
    if(ind==4) masterSpeed=atof(buffer);
    if(ind==5) masterHeading=atof(buffer);
    if(ind==6) choice=0;
    inputString="";
  }

  if(choice==2){

    if(ind==1) masterLat=atof(buffer);
    if(ind==2) masterLong=atof(buffer);
    // if(ind==3) myAltitude=atof(buffer);
    if(ind==3) masterAlt=atof(buffer);
    if(ind==4) masterSpeed=atof(buffer);
    if(ind==5) masterHeading=atof(buffer);
    if(ind==6) choice=0;
    inputString="";
  }
  if(choice==3){

    if(ind==1) myLat=atof(buffer);
    if(ind==2) myLong=atof(buffer);
    if(ind==3) myAltitude=atof(buffer);
    if(ind==4) pitchInput=atof(buffer);
    if(ind==5) rollInput=atof(buffer);
    if(ind==6) trueHeading=atof(buffer);
    if(ind==7) GpsSpeed=atof(buffer);
    if(ind==8) choice=0;
    inputString="";
  }

   if(choice==4){

    if(ind==1) trueHeading=atof(buffer);
    if(ind==2) kalmanLong=atof(buffer);
    if(ind==3) choice=0;
    inputString="";
  }
}

// void mavlink_pack_and_send(){
//   #define system_id 1
//     // Define the system type (see mavlink_types.h for list of possible types) 
//   int system_type = 1;//MAV_QUADROTOR;
//   int autopilot_type = 1;//MAV_AUTOPILOT_GENERIC;
  
//   // Initialize the required buffers 
//   mavlink_message_t msg; 
//   uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
//   // Pack the message
//   // mavlink_message_heartbeat_pack(system id, component id, message container, system type, MAV_AUTOPILOT_GENERIC)
//   mavlink_msg_heartbeat_pack(sys_id, 200, &msg, system_type, autopilot_type,1,1,1); 
//   // Copy the message to send buffer 
//   uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
//     // Send the message (.write sends as bytes) 
//   Serial3.write(buf, len);
//     /**
//    * @brief Pack a sys_status message
//    * @param system_id ID of this system
//    * @param component_id ID of this component (e.g. 200 for IMU)
//    * @param msg The MAVLink message to compress the data into
//    *
//    * @param onboard_control_sensors_present Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
//    * @param onboard_control_sensors_enabled Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
//    * @param onboard_control_sensors_health Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
//    * @param load Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
//    * @param voltage_battery Battery voltage, in millivolts (1 = 1 millivolt)
//    * @param current_battery Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
//    * @param battery_remaining Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
//    * @param drop_rate_comm Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
//    * @param errors_comm Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
//    * @param errors_count1 Autopilot-specific errors
//    * @param errors_count2 Autopilot-specific errors
//    * @param errors_count3 Autopilot-specific errors
//    * @param errors_count4 Autopilot-specific errors
//    * @return length of the message in bytes (excluding serial stream start sign)
//    */
//   mavlink_msg_sys_status_pack(sys_id, 200, &msg, 0, 0, 0, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0);
//   // Copy the message to send buffer 
//   len = mavlink_msg_to_send_buffer(buf, &msg);
//   // Send the message (.write sends as bytes) 
//   Serial3.write(buf, len);
//   *
//    * @brief Pack a gps_raw_int message
//    * @param system_id ID of this system
//    * @param component_id ID of this component (e.g. 200 for IMU)
//    * @param msg The MAVLink message to compress the data into
//    *
//    * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
//    * @param fix_type See the GPS_FIX_TYPE enum.
//    * @param lat Latitude (WGS84), in degrees * 1E7
//    * @param lon Longitude (WGS84), in degrees * 1E7
//    * @param alt Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.
//    * @param eph GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
//    * @param epv GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
//    * @param vel GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
//    * @param cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
//    * @param satellites_visible Number of satellites visible. If unknown, set to 255
//    * @return length of the message in bytes (excluding serial stream start sign)
   
//   mavlink_msg_gps_raw_int_pack(sys_id,220,&msg,micros(), 3,(projectedLat*10000000),(projectedLong*10000000), (myAltitude*1000),1,1,(GpsSpeed*100),trueHeading,GpsSat);
//   // Copy the message to send buffer 
//   len = mavlink_msg_to_send_buffer(buf, &msg);
//   // Send the message (.write sends as bytes) 
//   Serial3.write(buf, len);
//    // * @brief Pack a attitude message
//    // * @param system_id ID of this system
//    // * @param component_id ID of this component (e.g. 200 for IMU)
//    // * @param msg The MAVLink message to compress the data into
//    // *
//    // * @param time_boot_ms Timestamp (milliseconds since system boot)
//    // * @param roll Roll angle (rad, -pi..+pi)
//    // * @param pitch Pitch angle (rad, -pi..+pi)
//    // * @param yaw Yaw angle (rad, -pi..+pi)
//    // * @param rollspeed Roll angular speed (rad/s)
//    // * @param pitchspeed Pitch angular speed (rad/s)
//    // * @param yawspeed Yaw angular speed (rad/s)
//   mavlink_msg_attitude_pack(sys_id, 200, &msg, micros(), d2r(rollInput), d2r(pitchInput), d2r(trueHeading), 10.0, 10.00, 10.00);
//   // Copy the message to send buffer 
//   len = mavlink_msg_to_send_buffer(buf, &msg);
//   // Send the message (.write sends as bytes) 
//   Serial3.write(buf, len);

//     /**
//    * @brief Pack a altitude message
//    * @param system_id ID of this system
//    * @param component_id ID of this component (e.g. 200 for IMU)
//    * @param msg The MAVLink message to compress the data into
//    *
//    * @param time_usec Timestamp (micros since boot or Unix epoch)
//    * @param altitude_monotonic This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the local altitude change). The only guarantee on this field is that it will never be reset and is consistent within a flight. The recommended value for this field is the uncorrected barometric altitude at boot time. This altitude will also drift and vary between flights.
//    * @param altitude_amsl This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output AMSL by default and not the WGS84 altitude.
//    * @param altitude_local This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference to the coordinate origin (0, 0, 0). It is up-positive.
//    * @param altitude_relative This is the altitude above the home position. It resets on each change of the current home position.
//    * @param altitude_terrain This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller than -1000 should be interpreted as unknown.
//    * @param bottom_clearance This is not the altitude, but the clear space below the system according to the fused clearance estimate. It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving target. A negative value indicates no measurement available.
//    * @return length of the message in bytes (excluding serial stream start sign)
//    */
//    //mavlink_msg_altitude_pack(sys_id,200,&msg,micros(), float altitude_monotonic, float altitude_amsl, float altitude_local, float altitude_relative, float altitude_terrain, float bottom_clearance)
//   // Copy the message to send buffer 
// //  len = mavlink_msg_to_send_buffer(buf, &msg);
// //  // Send the message (.write sends as bytes) 
// //  Serial.write(buf, len);
//  }

 float d2r(float deg)
{
  float rad = deg*(3.14159265359/180);
  return rad;
}





#define         WGS84_A               6378137.0
#define         WGS84_ECCSQ           0.00669437999013
//#define         WGS84_A               6378137.0
//#define         WGS84_ECCSQ           0.00669438002290    //NAD

/* This routine determines the correct UTM letter designator for the given
   latitude and returns 'Z' if latitude is outside the UTM limits of 84N to 80S
   Written by Chuck Gantz- chuck.gantz@globalstar.com */

char UTMLetterDesignator(double Lat)
{
  char LetterDesignator;
  
  if((84 >= Lat) && (Lat >= 72)) LetterDesignator = 'X';
  else if((72 > Lat) && (Lat >= 64)) LetterDesignator = 'W';
  else if((64 > Lat) && (Lat >= 56)) LetterDesignator = 'V';
  else if((56 > Lat) && (Lat >= 48)) LetterDesignator = 'U';
  else if((48 > Lat) && (Lat >= 40)) LetterDesignator = 'T';
  else if((40 > Lat) && (Lat >= 32)) LetterDesignator = 'S';
  else if((32 > Lat) && (Lat >= 24)) LetterDesignator = 'R';
  else if((24 > Lat) && (Lat >= 16)) LetterDesignator = 'Q';
  else if((16 > Lat) && (Lat >= 8)) LetterDesignator = 'P';
  else if(( 8 > Lat) && (Lat >= 0)) LetterDesignator = 'N';
  else if(( 0 > Lat) && (Lat >= -8)) LetterDesignator = 'M';
  else if((-8> Lat) && (Lat >= -16)) LetterDesignator = 'L';
  else if((-16 > Lat) && (Lat >= -24)) LetterDesignator = 'K';
  else if((-24 > Lat) && (Lat >= -32)) LetterDesignator = 'J';
  else if((-32 > Lat) && (Lat >= -40)) LetterDesignator = 'H';
  else if((-40 > Lat) && (Lat >= -48)) LetterDesignator = 'G';
  else if((-48 > Lat) && (Lat >= -56)) LetterDesignator = 'F';
  else if((-56 > Lat) && (Lat >= -64)) LetterDesignator = 'E';
  else if((-64 > Lat) && (Lat >= -72)) LetterDesignator = 'D';
  else if((-72 > Lat) && (Lat >= -80)) LetterDesignator = 'C';
  else LetterDesignator = 'Z'; 
  return LetterDesignator;
}




void latLongToUtm(float Lat, float Long, float& UTMEasting, float& UTMNorthing, String& UTMZone) {
  double LongOrigin, LongOriginRad;
  double eccPrimeSquared;
  double k0 = 0.9996, N, T, C, A, M;
  double LatRad = Lat * M_PI / 180.0;
  double LongRad = Long * M_PI / 180.0;
  int ZoneNumber;

  ZoneNumber = (int)((Long + 180) / 6) + 1;
  
  if(Lat >= 56.0 && Lat < 64.0 && Long >= 3.0 && Long < 12.0)
    ZoneNumber = 32;
  
  // Special zones for Svalbard
  if(Lat >= 72.0 && Lat < 84.0) {
    if(Long >= 0.0  && Long <  9.0) ZoneNumber = 31;
    else if(Long >= 9.0  && Long < 21.0) ZoneNumber = 33;
    else if(Long >= 21.0 && Long < 33.0) ZoneNumber = 35;
    else if(Long >= 33.0 && Long < 42.0) ZoneNumber = 37;
  }
  // +3 puts origin in middle of zone
  LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3;  
  LongOriginRad = LongOrigin * M_PI / 180.0;

  // compute the UTM Zone from the latitude and longitude
//  sprintf(UTMZone, "%d%c", ZoneNumber, UTMLetterDesignator(Lat));
  //Stringstream sstr;
 //  Serial.print(" Zone Number: ");
 // Serial.print(ZoneNumber);Serial.println(UTMLetterDesignator(Lat));
  //UTMZone = sstr.str(); fizme

  eccPrimeSquared = WGS84_ECCSQ / (1 - WGS84_ECCSQ);
  N = WGS84_A / sqrt(1 - WGS84_ECCSQ * sin(LatRad) * sin(LatRad));
  T = tan(LatRad) * tan(LatRad);
  C = eccPrimeSquared * cos(LatRad) * cos(LatRad);
  A = cos(LatRad) * (LongRad-LongOriginRad);
  M = WGS84_A * ((1 - WGS84_ECCSQ / 4 - 3 * WGS84_ECCSQ * WGS84_ECCSQ / 64
            - 5 * WGS84_ECCSQ * WGS84_ECCSQ * WGS84_ECCSQ / 256) * LatRad 
           - (3 * WGS84_ECCSQ / 8 + 3 * WGS84_ECCSQ * WGS84_ECCSQ / 32
              + 45 * WGS84_ECCSQ * WGS84_ECCSQ * WGS84_ECCSQ / 1024) * 
           sin(2 * LatRad) + (15 * WGS84_ECCSQ * WGS84_ECCSQ / 256 +
                              45 * WGS84_ECCSQ * WGS84_ECCSQ * 
                              WGS84_ECCSQ / 1024) * sin(4 * LatRad) 
           - (35 * WGS84_ECCSQ * WGS84_ECCSQ * WGS84_ECCSQ / 3072) * 
           sin(6 * LatRad));
 UTMEasting = (double)(k0 * N * (A + (1 - T + C) * A * A * A / 6
                                   + (5 - 18 * T + T * T + 72 * C - 
                                      58 * eccPrimeSquared)* 
                                  A * A * A * A *A / 120) + 500000.0);
  UTMNorthing = (double)(k0 * (M + N * tan(LatRad) * 
                                (A * A / 2 + (5 - T + 9 * C + 4 * C * C)
                                 * A * A * A *A / 24
                                 + (61 - 58 * T + T * T + 
                                    600 * C - 330 * eccPrimeSquared) * 
                                 A * A * A * A * A * A / 720)));
  if(Lat < 0)
    UTMNorthing += 10000000.0; //10000000 meter offset for southern hemisphere
}

// void latLongToUtm(double Lat, double Long, double* UTMEasting, double* UTMNorthing, char* UTMZone) {
//   String zone;
//   latLongToUtm(Lat, Long, UTMEasting, UTMNorthing, zone);
//   strcpy(UTMZone, zone.c_str());
// }
  /* converts UTM coords to lat/long.  Equations from USGS Bulletin 1532 
     East Longitudes are positive, West longitudes are negative. 
     North latitudes are positive, South latitudes are negative
     Lat and Long are in decimal degrees. 
     Written by Chuck Gantz- chuck.gantz@globalstar.com */

void utmToLatLong(float UTMEasting, float UTMNorthing, const String& UTMZone, float& Lat,  float& Long)
{
  double k0 = 0.9996, eccPrimeSquared, N1, T1, C1, R1, D, M;
  double e1 = (1 - sqrt(1 - WGS84_ECCSQ))/(1 + sqrt(1 - WGS84_ECCSQ));
  double LongOrigin, mu, phi1, phi1Rad, x, y;
  int ZoneNumber, NorthernHemisphere; // 1 for northern hem., 0 for southern
  char* ZoneLetter;
  
  x = UTMEasting - 500000.0; /* remove 500,000 meter offset for longitude */
  y = UTMNorthing;
  
  ZoneNumber = strtoul(UTMZone.c_str(), &ZoneLetter, 10);
  if((*ZoneLetter - 'N') >= 0)
    NorthernHemisphere = 1;  /* point is in northern hemisphere */
  else {
    NorthernHemisphere = 0;  /* point is in southern hemisphere */
    y -= 10000000.0;         /* remove 10,000,000 meter offset 
                                used for southern hemisphere */
  }
  
  /* +3 puts origin in middle of zone */
  LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3;  
  
  eccPrimeSquared = (WGS84_ECCSQ) / (1 - WGS84_ECCSQ);
  
  M = y / k0;
  mu = M / (WGS84_A * (1 - WGS84_ECCSQ / 4 - 
                       3 * WGS84_ECCSQ * WGS84_ECCSQ / 64 - 5 * WGS84_ECCSQ * 
                       WGS84_ECCSQ * WGS84_ECCSQ / 256));
  phi1Rad = mu + (3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * sin(2 * mu) +
    (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * sin(4 * mu) +
    (151 * e1 * e1 * e1 / 96) * sin(6 * mu);
  phi1 = dgc_r2d(phi1Rad);
  
  N1 = WGS84_A / sqrt(1 - WGS84_ECCSQ * sin(phi1Rad) * sin(phi1Rad));
  T1 = tan(phi1Rad) * tan(phi1Rad);
  C1 = eccPrimeSquared * cos(phi1Rad) * cos(phi1Rad);
  R1 = WGS84_A * (1 - WGS84_ECCSQ) / 
    pow(1 - WGS84_ECCSQ * sin(phi1Rad) * sin(phi1Rad), 1.5);
  D = x / (N1 * k0);
  
  Lat = phi1Rad - (N1 * tan(phi1Rad) / R1) * 
    (D * D / 2 - (5 + 3 * T1 + 10 * C1 - 4 * C1 * C1 - 9 * eccPrimeSquared) * 
     D * D * D * D / 24 +
     (61 + 90 * T1 + 298 * C1 + 45 * T1 * T1 - 
      252 * eccPrimeSquared - 3 * C1 * C1) * D * D * D * D * D * D / 720);
  Lat = dgc_r2d(Lat);

  Long = (D - (1 + 2 * T1 + C1) * D * D * D / 6 + 
           (5 - 2 * C1 + 28 * T1 - 3 * C1 * C1 + 
            8 * eccPrimeSquared + 24 * T1 * T1)
          * D * D * D * D * D / 120) / cos(phi1Rad);
  Long = LongOrigin + dgc_r2d(Long);
}

// void utmToLatLong(float UTMEasting, float UTMNorthing, const char UTMZone, float& Lat,  float& Long) {
//   String zone = UTMZone;
//   utmToLatLong(UTMEasting, UTMNorthing, zone, Lat, Long);
// }




inline float dgc_r2d(float theta) {
    return (theta * 180.0 / M_PI);
}

inline float dgc_d2r(float theta) {
    return (theta * M_PI / 180.0);
}

inline float dgc_r2df(float theta) {
    return (theta * 180.0 / M_PI);
}


// // Example variable, by declaring them static they're persistent
// // and will thus track the system state
// static int packet_drops = 0;
// // static int mode = MAV_MODE_UNINIT;  Defined in mavlink_types.h, which is included by mavlink.h 
 
// /**
// * @brief Receive communication packets and handle them
// *
// * This function decodes packets on the protocol level and also handles
// * their value by calling the appropriate functions.
// */
// // mavlink_status_t status;
// // static void communication_receive(void)
// // {
// //   mavlink_message_t msg;
// //   mavlink_status_t status;
 
// //   // COMMUNICATION THROUGH EXTERNAL UART PORT (XBee serial)
 
// //   if(Serial3.available())
// //   {
// //     uint8_t c = Serial3.read();
// //     // Try to get a new message
// //     if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
// //       // Handle message
 
// //       switch(msg.msgid)
// //       {
// //               case MAVLINK_MSG_ID_SET_MODE:
// //               {
// //           mode = mavlink_msg_set_mode_get_mode(&msg);
// //               }
// //               break;
// //       case MAVLINK_MSG_ID_ACTION:
// //         // EXECUTE ACTION
// //         break;
// //       default:
// //         //Do nothing
// //         break;
// //       }
// //     }
 
// //     // And get the next one
// //   }
 
// //   // Update global packet drops counter
// //   packet_drops += status.packet_rx_drop_count;
// //  }
// void communication_receive() { 
//   mavlink_message_t msg; 
//   mavlink_status_t status;
  
//   //receive data over serial 
//   while(Serial.available() > 0) { 
//     uint8_t c = Serial.read();
    
//     //try to get a new message 
//     if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) { 
//       // Handle message
//       switch(msg.msgid) {
//               case MAVLINK_MSG_ID_SET_MODE: {
//                 // set mode
//               }
//               break;
//               // case MAVLINK_MSG_ID_ACTION:
//           // EXECUTE ACTION
//         break;
//         default:
//           //Do nothing
//         break;
//       }
//     } 
//     // And get the next one
//   }
// }

//pitch .08 
//roll .2

void takeoff(){

  targetHeading =  headingOffset;

  pitchKp= 0.08 ;
  rollKp= 0.2;

  userollKp=1;
  userollKi=0;
  userollKd=0;
  usepitchKp=1;
  usepitchKi=0;
  usepitchKd=0;

  pitchSetpoint =20;

  while(takeoff_flag)
  {

   tiempo=millis();
   UpdateOrientation();

  float headingError =  angular_diff( targetHeading,trueHeading);
  rollSetpoint = controlPID(headingError,headingKp,headingKi,headingKd,1,0,1,40,40,40,0,headingAccum, headingErrorOld,headingError);
   //Saturation point on roll rollMaxAllowed set in header
  if (rollSetpoint > rollMaxAllowed) rollSetpoint=rollMaxAllowed;
  if (rollSetpoint < -1*rollMaxAllowed) rollSetpoint=-1*rollMaxAllowed;

   if ((counter%4)==0) Navigation();
   timetook = tiempo-timetravel;
    notify_user();//----------
   if ((counter%5)==0) correctgyroheading();
   if ((counter%15)==0) notify_swarm();
   recvWithStartEndMarkers();
   recvWithStartEndMarkers2();
   counter++;
   if(tiempo>10000) takeoff_flag=0;
   while(millis()<(tiempo+delayTime)) {
       recvWithStartEndMarkers();
       // communication_receive();
       recvWithStartEndMarkers2();
   }
    timetravel=tiempo;
}

}

