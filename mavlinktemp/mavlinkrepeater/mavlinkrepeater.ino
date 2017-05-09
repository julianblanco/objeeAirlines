#include "clib/common/mavlink.h" 

bool messagecomplete = false;
int stringComplete=0;
float myLat =0;
float myLong=0;
float myAltitude=0;
float pitchInput=0;
float rollInput=0;
float trueHeading=0;
float GpsSpeed=0;
String inputString ="";
float masterLat=0;
float masterLong=0;
float masterAlt=0;
float masterSpeed=0;
float masterHeading=0;
int sys_id =1;

int GpsSat=0;



void setup()
{
Serial.begin(9600);
}


void loop()
{
recvWithStartEndMarkers();
if (messagecomplete) mavlink_pack_and_send();
delayMicroseconds(10);
}



void mavlink_pack_and_send(){
  #define system_id 1
    // Define the system type (see mavlink_types.h for list of possible types) 
  int system_type = 1;//MAV_QUADROTOR;
  int autopilot_type = 1;//MAV_AUTOPILOT_GENERIC;
  
  // Initialize the required buffers 
  mavlink_message_t msg; 
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  // Pack the message
  // mavlink_message_heartbeat_pack(system id, component id, message container, system type, MAV_AUTOPILOT_GENERIC)
  mavlink_msg_heartbeat_pack(sys_id, 200, &msg, system_type, autopilot_type,1,1,1); 
  // Copy the message to send buffer 
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    // Send the message (.write sends as bytes) 
  Serial.write(buf, len);
    /**
   * @brief Pack a sys_status message
   * @param system_id ID of this system
   * @param component_id ID of this component (e.g. 200 for IMU)
   * @param msg The MAVLink message to compress the data into
   *
   * @param onboard_control_sensors_present Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
   * @param onboard_control_sensors_enabled Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
   * @param onboard_control_sensors_health Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
   * @param load Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
   * @param voltage_battery Battery voltage, in millivolts (1 = 1 millivolt)
   * @param current_battery Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
   * @param battery_remaining Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
   * @param drop_rate_comm Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
   * @param errors_comm Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
   * @param errors_count1 Autopilot-specific errors
   * @param errors_count2 Autopilot-specific errors
   * @param errors_count3 Autopilot-specific errors
   * @param errors_count4 Autopilot-specific errors
   * @return length of the message in bytes (excluding serial stream start sign)
   */
  mavlink_msg_sys_status_pack(sys_id, 200, &msg, 0, 0, 0, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0);
  // Copy the message to send buffer 
  len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes) 
  Serial.write(buf, len);
  /**
   * @brief Pack a gps_raw_int message
   * @param system_id ID of this system
   * @param component_id ID of this component (e.g. 200 for IMU)
   * @param msg The MAVLink message to compress the data into
   *
   * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
   * @param fix_type See the GPS_FIX_TYPE enum.
   * @param lat Latitude (WGS84), in degrees * 1E7
   * @param lon Longitude (WGS84), in degrees * 1E7
   * @param alt Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.
   * @param eph GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
   * @param epv GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
   * @param vel GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
   * @param cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
   * @param satellites_visible Number of satellites visible. If unknown, set to 255
   * @return length of the message in bytes (excluding serial stream start sign)
   */
  mavlink_msg_gps_raw_int_pack(sys_id,220,&msg,micros(), 3,(myLat*10000000),(myLong*10000000), (myAltitude*1000),1,1,(GpsSpeed*100),trueHeading,GpsSat);
  // Copy the message to send buffer 
  len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes) 
  Serial.write(buf, len);
   // * @brief Pack a attitude message
   // * @param system_id ID of this system
   // * @param component_id ID of this component (e.g. 200 for IMU)
   // * @param msg The MAVLink message to compress the data into
   // *
   // * @param time_boot_ms Timestamp (milliseconds since system boot)
   // * @param roll Roll angle (rad, -pi..+pi)
   // * @param pitch Pitch angle (rad, -pi..+pi)
   // * @param yaw Yaw angle (rad, -pi..+pi)
   // * @param rollspeed Roll angular speed (rad/s)
   // * @param pitchspeed Pitch angular speed (rad/s)
   // * @param yawspeed Yaw angular speed (rad/s)
  mavlink_msg_attitude_pack(sys_id, 200, &msg, micros(), d2r(rollInput), d2r(pitchInput), d2r(trueHeading), 10.0, 10.00, 10.00);
  // Copy the message to send buffer 
  len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes) 
  Serial.write(buf, len);

    /**
   * @brief Pack a altitude message
   * @param system_id ID of this system
   * @param component_id ID of this component (e.g. 200 for IMU)
   * @param msg The MAVLink message to compress the data into
   *
   * @param time_usec Timestamp (micros since boot or Unix epoch)
   * @param altitude_monotonic This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the local altitude change). The only guarantee on this field is that it will never be reset and is consistent within a flight. The recommended value for this field is the uncorrected barometric altitude at boot time. This altitude will also drift and vary between flights.
   * @param altitude_amsl This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output AMSL by default and not the WGS84 altitude.
   * @param altitude_local This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference to the coordinate origin (0, 0, 0). It is up-positive.
   * @param altitude_relative This is the altitude above the home position. It resets on each change of the current home position.
   * @param altitude_terrain This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller than -1000 should be interpreted as unknown.
   * @param bottom_clearance This is not the altitude, but the clear space below the system according to the fused clearance estimate. It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving target. A negative value indicates no measurement available.
   * @return length of the message in bytes (excluding serial stream start sign)
   */
   //mavlink_msg_altitude_pack(sys_id,200,&msg,micros(), float altitude_monotonic, float altitude_amsl, float altitude_local, float altitude_relative, float altitude_terrain, float bottom_clearance)
  // Copy the message to send buffer 
//  len = mavlink_msg_to_send_buffer(buf, &msg);
//  // Send the message (.write sends as bytes) 
//  Serial.write(buf, len);
  messagecomplete =false;
 }

 float d2r(float deg)
{
  float rad = deg*(3.14159265359/180);
  return rad;
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
    if ( Serial.available() ) {
        rc =  Serial.read();
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
    if(ind==6) {
      choice=0;
    messagecomplete=true;
  }
    inputString="";
  }

  if(choice==2){

    if(ind==1) masterLat=atof(buffer);
    if(ind==2) masterLong=atof(buffer);
    // if(ind==3) myAltitude=atof(buffer);
    if(ind==3) masterAlt=atof(buffer);
    if(ind==4) masterSpeed=atof(buffer);
    if(ind==5) masterHeading=atof(buffer);
    if(ind==6) {
      choice=0;
    messagecomplete=true;
  }
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
    if(ind==8) {
      choice=0;
    messagecomplete=true;
  }
    inputString="";
  }


 
}