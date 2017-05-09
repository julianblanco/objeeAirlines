#include "wiring_private.h" // pinPeripheral() function
#define PIN 13

#if defined(ARDUINO_ARCH_SAMD)
Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}
#endif

// #define bmpbarometer
#define pololubarmoneter

#include <SD.h>
#include "variables.h"

#define mySerial Serial1
GNSS GNSS(&mySerial);


// Declare the gosh darn function
void SDinit();

Servo leftElevon;
Servo rightElevon;
Servo throttle;


//Adafruit_BMP280 bme(BMP_CS); // hardware SPI
Adafruit_BNO055 bno = Adafruit_BNO055();
// Adafruit_BMP085 bmp;
LPS psbaro;


uint32_t counter=0;
//#define delayTime 33
#define delayTime 10
void setup(void) 
{
  pinMode(PIN, OUTPUT);
  servoBegin(leftElevon,rightElevon,throttle);
  CenterAll(leftElevon,rightElevon,throttle);
  delay(1000);
  
  Wire.begin();  // Initialize I2C bus
  Serial.begin(9600);
  GNSS.begin(9600);
  mySerial.begin(9600);
  // Assign pins 10 & 11 SERCOM functionality
  
  // XBEE.begin(9600);
  Serial4.begin(9600);
  #if defined(ARDUINO_ARCH_SAMD)
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);
  #endif
 Serial4.println("ObjeeAirlines Test");
 
  SDinit();

  //numOfWaypoints=(int) (sizeof(wplat)/sizeof(float));
  waypointing(1); // curious, cause of double waypoint startup?
  CenterAll(leftElevon,rightElevon,throttle);
  if(!bno.begin())while(1){};
  bno.setExtCrystalUse(true);

Serial4.println("GYRO STAT");
  //Barmoneter Intialzation****************************
  #ifdef bmpbarometer
  if (!bmp.begin())while (1) {};  
  intialpressure=bmp.readPressure();
  intialAltitude=bmp.readAltitude();
  #endif
  #ifdef pololubarmoneter
   if (!psbaro.init()) while (1);
  psbaro.enableDefault();
  intialpressure=averagebarmoeter(psbaro);
  #endif
  //***************************************************
Serial4.println("BAROMETER STAT");

  ServoWiggle(leftElevon);
  sampleGyro(bno);

  CenterAll(leftElevon,rightElevon,throttle);
  ServoWiggle(leftElevon);// Initialize servos
  CenterAll(leftElevon,rightElevon,throttle);
  uint8_t system, gyro, accel, mag = 0;
 
  // while(system != 3)
  // {
  //    delay(500);
  //    bno.getCalibration(&system, &gyro, &accel, &mag);

  // }

   ServoInit(leftElevon,rightElevon);  // Initialize servos
   imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
   headingOffset= euler.x();
    Serial4.println("Start");

}//end setup
unsigned long tiempo=0;


void loop(void)
{
  tiempo=millis();

   GNSS.read();//----------
   if ((counter%10)==0)Serial_Sample(GNSS);//----------]

  // Plane stabilization 2.4ms
  sampleGyro(bno);
  controlTactics();
  controlResponseWing(targetHeading, rollTargetValue, pitchSetpoint, trueHeading, rollInput, pitchInput);
  updateServos(leftElevon,rightElevon,throttle);
  //  recvWithStartEndMarkers(Serial2,mySerial);
  if ((counter%150)==0) samplebarometer(psbaro);
  //if ((counter%20)==0) IIRFilter();
  if ((counter%2)==0) Navigation();

  // // // Print to xbee serial
  if ((counter%400)==0) notify_user(Serial4);//----------
  // Save data to SD card
 if ((counter%30)==0) sdlog();//----------
 // digitalWrite(PIN, LOW);
 //delayTime = 33 so should be 30 hz
 counter++;
 // while(millis()<(tiempo+delayTime)) {
 //  digitalWrite(PIN, LOW);
 //   char c = GNSS.read();					 
 //   Serial_Sample(GNSS);
 // }
 // digitalWrite(PIN, HIGH);
}//end main






//Why is this here?
File myFile;
File myFile1;
File myFile2;
char nameoffile[16];
char nameoffile1[16];
char nameoffile2[16];
//
void sdlog()
{
     myFile = SD.open(nameoffile, FILE_WRITE);
     if (myFile) {
        myFile.print(myLat,6);myFile.print(',');
        myFile.print(myLong,6);myFile.print(',');
        myFile.print(targetLat,6);myFile.print(',');
        myFile.print(targetLong,6);myFile.print(',');
        myFile.print(targetHeading,2);myFile.print(',');
        myFile.print(Gpsheading,2);myFile.print(',');
        myFile.print(trueHeading,2);myFile.print(',');
        myFile.print(GpsSpeed,2);myFile.print(',');
        myFile.print(waypointNumber,1);myFile.print(',');
        myFile.print(rollInput,2);myFile.print(',');
        myFile.print(pitchInput,2);myFile.print(',');
        myFile.print(rollServoOutput,1);myFile.print(',');
        myFile.print(pitchServoOutput,1);myFile.print(',');
        myFile.print(headingSetpoint,2);myFile.print(',');
        myFile.print(rollTargetValue,2);myFile.print(',');
        myFile.print(pitchSetpoint,2);myFile.print(',');
        myFile.print(throtSetpoint,2);myFile.print(',');
        myFile.print(myAltitude,2);myFile.print(',');
        myFile.print(targetAlt,1);myFile.print(',');
        myFile.println(tiempo,1);
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
  // Serial.println(n);
  // Serial.println(nameoffile);
  //now nameoffile[] contains the name of a file that doesn't exist
   myFile= SD.open(nameoffile,FILE_READ);
   myFile.close();
   myFile= SD.open(nameoffile,FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {
    //Serial2.print("objeeAirlines Autonomous Veichile Test");
    myFile.println("latitude,longitude");
    // close the file:
    myFile.close();
    //Serial2.println("done.");
  } else {
    // if the file didn't open, print an error:
    //Serial2.println("error opening test.txt");
  }
}



// void errorlog()
// {
//      myFile = SD.open(nameoffile, FILE_WRITE);
//      if (myFile) {



//      }
// }
// void waypointread()
// {
//      myFile = SD.open("waypoints.txt", FILE_READ);
//      if (myFile) {
//         int idx = 0;
//           char instring = myFile.readln();
//           if instring
//           const char *p = inputString.c_str();

//           // p = strchr(p, ',')+1;
//           wplat[idx] = atof(p);
//           p = strchr(p, ',')+1;
//           if (',' != *p)
//           {
//            wplong[idx] = atof(p);
//           }




//      }
// }