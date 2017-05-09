
//#define Feather
#include "ServoM.h"
#include "Sensor.h"
#include "IMU_BNO055.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_GNSS.h"
#include "Craft.h"

#define mySerial Serial1
Adafruit_GPS GPS(&mySerial);


// Create servo objects
Servo rollServo;
Servo pitchServo;
Servo yawServo;

rollServo.attach(9);
yawServo.attach(10);
pitchServo.attach(11);

// Initialize servo classes
Control_Surface aileron ("aileron", rollServo, 9, 1500, 1100, 1900);
Control_Surface elevator ("elevator", pitchServo, 10, 1500, 1100, 1900);
Control_Surface rudder ("rudder", yawServo, 11, 1500, 1100, 1900);

Control_Surface control_surfaces[3] = { aileron, elevator, rudder };


Craft airplane();


Adafruit_BNO055 bno = Adafruit_BNO055();

uint32_t counter = 0;
#define delayTime 10

void setup(void)
{
	// ISRinit();
	Wire.begin();  // Initialize I2C bus
	Serial.begin(9600);
	pinMode(13, OUTPUT);

	Serial.println("objeeAirlines Autonomous Veichile Log File");
	GPS.begin(9600);
	mySerial.begin(9600);
	//waypointing(); // curious, cause of double waypoint startup?
	//trueHeading= 0; // curious, initialize with magnetic? 




if(!bno.begin())
	{
	/* There was a problem detecting the BNO055 ... check your connections */
	Serial.print("Error: BNO055 not detected");
	while(1);
	}

	bno.setExtCrystalUse(true);
	SampleGyro();
	while(calibration != 3)
	{
	 delay(500);
	 SampleGyro();
	}
	ServoInit();  // Initialize servos
	Serial_Sample();
	yawOffset= 0-yawInput;
}







void loop(void) 
{
	uint32_t startTime=millis();

	// Poll serial port for GPS 
	char c = GPS.read();
	Serial_Sample();

	// Call functionality at specific frequency depending on loop speed
	if (counter%10)  UpdateOrientation();
	if (counter%5) SampleGyro();
	if (counter%200)  IIRFilter();
	if (counter%200) Navigation();
	if (counter%10000) notify_user();
	if (counter%10)  UpdateServos();
	counter++;

	// delay to control looping frequency
	while(millis()<startTime+delayTime) char c = GPS.read();
}



void Serial_Sample(void) {

    if (GPS.newNMEAreceived()) {
        if (GPS.parse(GPS.lastNMEA()))  // this also sets the newNMEAreceived() flag to false
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
}



void SampleGyro()

{
	/* Display the floating  point data */
	imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
	yawInput=euler.x()+yawOffset;
	pitchInput=euler.y();
	rollInput=fmod((euler.z()+(360+90)), 360)-180;
	uint8_t system, gyro, accel, mag = 0;
	bno.getCalibration(&system, &gyro, &accel, &mag);
	calibration=system;
}

void UpdateServos() {
     
	rollServo.writeMicroseconds(rollServoOutput);
	pitchServo.writeMicroseconds(pitchServoOutput);
	yawServo.writeMicroseconds(yawServoOutput);
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