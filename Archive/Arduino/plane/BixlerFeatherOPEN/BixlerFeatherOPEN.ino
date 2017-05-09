//#define pitot
#define barometer


#include "wiring_private.h" // pinPeripheral() function


#define PIN 13

// #ifdef _VARIANT_ARDUINO_ZERO_
// volatile uint32_t *setPin = &PORT->Group[g_APinDescription[PIN].ulPort].OUTSET.reg;
// volatile uint32_t *clrPin = &PORT->Group[g_APinDescription[PIN].ulPort].OUTCLR.reg;

// const uint32_t  PinMASK = (1ul << g_APinDescription[PIN].ulPin);
// #endif


Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}

#include <SD.h>
//#define sonar
//#define takeoff
//#define landing
//#define fullScale
//XBEE
//Serial 5 on Feather Digital 6 and 7
//#define serialCommands
#define xbeeSerial
#define sdcard
#include "variables.h"

#define mySerial Serial1
GNSS GNSS(&mySerial);

#ifdef xbeeSerial
 // #define xbee Serial2
#endif

Servo aileron;
Servo elevator;
Servo rudder;
Servo Throttle;

Adafruit_BNO055 bno = Adafruit_BNO055();
Adafruit_BMP085 bmp;

uint32_t counter=0;
#define delayTime 50

void setup(void) 
{
   pinMode(PIN, OUTPUT);
  servoBegin(aileron,elevator,rudder,Throttle);
 
  CenterAll(aileron,elevator,rudder,Throttle);
  delay(1000);
  
  Wire.begin();  // Initialize I2C bus
  Serial.begin(9600); 
  GNSS.begin(9600);
  mySerial.begin(9600);
 
  #ifdef xbeeSerial

   Serial2.begin(9600);
     // Assign pins 10 & 11 SERCOM functionality
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);
 //  inputString.reserve(200);
  Serial2.println("ObjeeAirlines Test");
  #endif
  
  #ifdef sdcard
  // xbee.print("Initializing SD card...");
  SDinit();
  #endif

  waypointing(1); // curious, cause of double waypoint startup?
  //xbee.println("objeeAirlines Autonomous Veichile Log File");
  CenterAll(aileron,elevator,rudder,Throttle);
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
  CenterAll(aileron,elevator,rudder,Throttle);
  ServoWiggle(aileron,elevator,rudder);// Initialize servos
  CenterAll(aileron,elevator,rudder,Throttle);
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





int controlTest =1;

void loop(void)
{
  uint32_t tiempo=millis();

  char c = GNSS.read();
  Serial_Sample(GNSS);

  
  if (counter%5) SampleGyro(bno);
  if (counter%20)  IIRFilter();
  if (counter%20) Navigation();
  if (counter%50) notify_user(Serial2);
 // if (counter%1000) xbee();
  if (counter%2)  UpdateServos(aileron,elevator,rudder,Throttle);
  if (counter%10)  samplebarometer(bmp);
  if (counter%3) sdlog();
  if(counter%3)  serialEvent(Serial2);
  if(counter%60) stringChecker(Serial2);
  if(counter>300&&counter<350)
    {
    controlTest =1;
    }
    else
    {
      controlTest =0;
    }
     if (counter%10)  UpdateOrientation(controlTest);
  counter++;
  //delayTime =10 so should be 100 hz
  while(millis()<tiempo+delayTime){char c = GNSS.read();Serial_Sample(GNSS);
  }

    static int x=1;
        if(x==1){
      digitalWrite(PIN, HIGH);
      x=0;}
      else{
      digitalWrite(PIN, LOW);
      x=1;
    }


}




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
        myFile.print(headingError,2);myFile.print(',');
        myFile.print(GpsSpeed,2);myFile.print(',');
        myFile.print(waypointNumber,1);myFile.print(',');
        myFile.print(distanceToTarget,4);myFile.print(',');
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
        myFile.print(myAltitude,2);myFile.print(',');
        myFile.print(targetAlt,1);myFile.print(',');
        myFile.print(pressure,1);myFile.print(',');//myFile.print(',');
        myFile.print(distanceXT,1);myFile.print(',');//myFile.print(',');
        //Add gps time
        myFile.println(controlTest,1);//25th
       
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
    myFile.println("objeeAirlines Autonomous Vehicle Test");
    // close the file:
    myFile.close();
    //Serial2.println("done.");
  } else {
    // if the file didn't open, print an error:
    //Serial2.println("error opening test.txt");
  }
}

