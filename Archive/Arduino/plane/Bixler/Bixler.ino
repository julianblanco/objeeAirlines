#define DUE

#include "variables.h"
#include "platform.h"

 #define mySerial Serial1
 Adafruit_GPS GPS(&mySerial);
#define mySerial2 Serial2
 i2cSlave i2cSlave(&mySerial2);

Servo rollServo;
Servo pitchServo;
Servo yawServo;

void setup(void) 
{
  Wire.begin();  // Initialize I2C bus
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  startTimer(TC1, 0, TC3_IRQn, 2000); 
  Serial.println("objeeAirlines Autonomous Veichile Log File");
  GPS.begin(9600);
  mySerial.begin(9600);
  i2cSlave.ibegin(115200);
  mySerial2.begin(115200);
  waypointing(); // curious, cause of double waypoint startup?
  trueHeading= 0; // curious, initialize with magnetic? 



rollServo.attach(9);
yawServo.attach(10);
pitchServo.attach(11);


  while(calibration != 3)
  {
     delay(500);
     Serial_Sample();
  }
  ServoInit();  // Initialize servos
  Serial_Sample();
  yawoffset= 0-yawInput; 

}

void loop(void) 
{

  Serial_Sample();
  if (orient_update)  UpdateOrientation();
  if (iir_update)  IIRFilter();
  if (navigate_update) Navigation();
  if (User_update) notify_user();
  if (servo_update)  UpdateServos();

}

void Serial_Sample(void) {

    if (GPS.newNMEAreceived()) {
        if (GPS.parse(GPS.lastNMEA()))                          // this also sets the newNMEAreceived() flag to false
        {
            new_GPS_data = 1;
            myLat=GPS.latitudeDegrees;
            myLong=GPS.longitudeDegrees;
            GpsSpeed=GPS.speed;
            Gpsheading=GPS.angle;
            GpsAltitude=GPS.altitude;
            GpsSat=GPS.satellites;
        }
  
        Fix=GPS.fix;
    }
 
   // if a sentence is received, we can check the checksum, parse it...
   if (i2cSlave.inewNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (i2cSlave.iparse(i2cSlave.ilastNMEA())){   // this also sets the newNMEAreceived() flag to false
      yawInput= i2cSlave.xdeg+yawOffset;
     pitchInput=i2cSlave.ydeg;
     rollInput=i2cSlave.zdeg;
     calibration=i2cSlave.calibration;
    //  return;  // we can fail to parse a sentence in which case we should just wait for another
      }
  }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
  gps_update=0;
}


void UpdateServos() {
     
  rollServo.writeMicroseconds(rollServoOutput);
  pitchServo.writeMicroseconds(pitchServoOutput);
  yawServo.writeMicroseconds(yawServoOutput);
  servo_update=0;
}

void ServoInit() {
  // Initialize control values


  rollServo.writeMicroseconds(1500);
  pitchServo.writeMicroseconds(1500);
  yawServo.writeMicroseconds(1500);
  delay(500);

  rollServo.writeMicroseconds(1900);
  delay(500);
  pitchServo.writeMicroseconds(1900);
  rollServo.writeMicroseconds(1500);

  delay(500);
  pitchServo.writeMicroseconds(1500);
  yawServo.writeMicroseconds(1900);
  rollServo.writeMicroseconds(1100);
  delay(500);
  yawServo.writeMicroseconds(1500);
  pitchServo.writeMicroseconds(1100);
  delay(500);
  yawServo.writeMicroseconds(1100);
  pitchServo.writeMicroseconds(1500);
  delay(1000);
  yawServo.writeMicroseconds(1500);
}



void CenterAll() {
  // Center all servos
  rollServo.writeMicroseconds(1500);
  pitchServo.writeMicroseconds(1500);
  yawServo.writeMicroseconds(1500);
}

void TC3_Handler()
{
  TC_GetStatus(TC1, 0);
  char c = GPS.read();  
  char d = i2cSlave.iread();
  ISR_count++;
  if ((ISR_count % 200) == 0)  orient_update = 1;  // Update orientation data
  if ((ISR_count % 500) == 0)  iir_update=1;
  if ((ISR_count % 100) == 0)  servo_update = 1;  //
  if ((ISR_count % 800) == 0)  navigate_update=1;
  if ((ISR_count % 1000) == 0) User_update= 1;
  if (ISR_count >= 2000)  ISR_count = 0;  // Reset ISR_count
 }