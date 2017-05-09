//#define pitot
#define barometer
//#define sonar
//#define takeoff
//#define landing
//#define fullScale
//XBEE
//Serial 5 on Feather Digital 6 and 7
//#define serialCommands
//#define xbeeSerial
#define sdcard
#include "variables.h"

#define mySerial Serial1
GNSS gnss(&mySerial);








#ifdef xbeeSerial
  #define xbee Serial5
#endif

Servo aileron;
Servo elevator;
Servo rudder;

Adafruit_BNO055 bno = Adafruit_BNO055();
Adafruit_BMP085 bmp;

uint32_t counter=0;
#define delayTime 100

void setup(void) 
{
  servoBegin(aileron,elevator,rudder);
  Wire.begin();  // Initialize I2C bus
  Serial.begin(9600); 
  gnss.begin(9600);
  mySerial.begin(9600);

  #ifdef xbeeSerial

    xbee.begin(9600);
  #endif
#ifdef sdcard
 // xbee.print("Initializing SD card...");
SDinit();
#endif

  waypointing(); // curious, cause of double waypoint startup?
//xbee.println("objeeAirlines Autonomous Veichile Log File");
 CenterAll(aileron,elevator,rudder);
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    //xbee.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1){};
  }

  #ifdef barometer
    if (!bmp.begin()) {
    //xbee.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {};
  }
  intialpressure=bmp.readPressure();
  intialAltitude=bmp.readAltitude();

  #endif

  #ifdef fullScale
  CenterAll(aileron,elevator,rudder);
  ServoWiggle(aileron,elevator,rudder);
  while(Fix==0)
  {
      Serial_Sample(mySerial);
  }
  #endif
 
  bno.setExtCrystalUse(true);
  SampleGyro(bno);
  CenterAll(aileron,elevator,rudder);
  ServoWiggle(aileron,elevator,rudder);// Initialize servos
  CenterAll(aileron,elevator,rudder);
  while(calibration != 3)
  {
     delay(500);
     SampleGyro(bno);
  }
  ServoInit(aileron,elevator,rudder);  // Initialize servos
   imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
   yawOffset= euler.x(); //yax
 // yawOffset=SampleGyroX();
}







void loop(void)
{
  uint32_t tiempo=millis();

  char c = gnss.read();
  Serial_Sample(gnss);

  if (counter%10)  UpdateOrientation();
  if (counter%5) SampleGyro(bno);
  if (counter%200)  IIRFilter();
  if (counter%20) Navigation();
  if (counter%10000) notify_user();
  if (counter%10)  UpdateServos(aileron,elevator,rudder);
  #ifdef barometer
  if (counter%10)  samplebarometer(bmp);
  #endif
  if (counter%10000) sdlog();
  counter++;
  //delayTime =10 so should be 100 hz
  while(millis()<tiempo+delayTime){char c = gnss.read();Serial_Sample(gnss);}

}





