#include <I2C.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"

#include <SD.h>
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

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (1)

Adafruit_BNO055 bno = Adafruit_BNO055();


float x;
float y;
float z;

float eulerx;
float eulery;
float eulerz;
int calsystem;
int calgyro;
int calaccel;
int calmag ;
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

int distance;

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  pinMode(13,OUTPUT);
  Serial.begin(9600);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");
  digitalWrite(13,LOW);
  I2c.begin(); // Opens & joins the irc bus as master
  delay(100); // Waits to make sure everything is powered up before sending or receiving data  
  I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails

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

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  // SDinit();
  digitalWrite(13,HIGH);
  uint8_t system, gyro, accel, mag = 0;
   while(system != 3)
  {
     delay(500);
     bno.getCalibration(&system, &gyro, &accel, &mag);

  }

  digitalWrite(13,LOW);
  delay(1500);
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
 unsigned long counter = 0;
void loop(void)
{

  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // Write 0x04 to register 0x00
  if ((counter%50)==0){
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.write(LIDARLite_ADDRESS,RegisterMeasure, MeasureValue); // Write 0x04 to 0x00
    delay(1); // Wait 1 ms to prevent overpolling
  }

  byte distanceArray[2]; // array to store distance bytes from read function
  
  // Read 2byte distance from register 0x8f
  nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.read(LIDARLite_ADDRESS,RegisterHighLowB, 2, distanceArray); // Read 2 Bytes from LIDAR-Lite Address and store in array
    delay(1); // Wait 1 ms to prevent overpolling
  }
  distance = (distanceArray[0] << 8) + distanceArray[1];  // Shift high byte [0] 8 to the left and add low byte [1] to create 16-bit int
  // - VECTOR_GRAVITY       - m/s^2
}

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  eulerx =euler.x();
  eulery =euler.y();
  eulerz =euler.z();


  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  calsystem =system;
  calgyro=gyro;
  calaccel = accel;
  calmag = mag;
  
  imu::Vector<3> linacceel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  x =linacceel.x();
  y =linacceel.y();
  z =linacceel.z();


  sdlog();
  // digitalWrite(13,HIGH);
  delay(BNO055_SAMPLERATE_DELAY_MS);
  // digitalWrite(13,LOW);
}





// //Why is this here?
// File myFile;
// File myFile1;
// File myFile2;
// char nameoffile[16];
// char nameoffile1[16];
// char nameoffile2[16];
// //
void sdlog()
{
     // myFile = SD.open(nameoffile, FILE_WRITE);
     // if (myFile) {
        Serial.print(x,6);Serial.print(',');
        Serial.print(y,6);Serial.print(',');
        Serial.print(z,6);Serial.print(',');
        Serial.print(eulerx,2);Serial.print(',');
        Serial.print(eulery,2);Serial.print(',');
        Serial.print(eulerz,2);Serial.print(',');
        Serial.print(calsystem);Serial.print(',');
        Serial.print(calaccel);Serial.print(',');
        Serial.print(calgyro);Serial.print(',');
        Serial.print(calmag);Serial.print(',');
        Serial.print(distance);Serial.print(',');
        Serial.println(millis());
      // }
       // myFile.close();
}


// void SDinit()
// {
//   if (!SD.begin(4)) {
//    // xbee.println("initialization failed!");
//     return;
//   }
//   //xbee.println("initialization done.");

//   // open the file. note that only one file can be open at a time,
//   // so you have to close this one before opening another.
//    // make it long enough to hold your longest file name, plus a null terminator
//   int n = 0;
//   snprintf(nameoffile, sizeof(nameoffile), "data%03d.txt", n); // includes a three-digit sequence number in the file name
//   while(SD.exists(nameoffile)) {
//     n++;
//     snprintf(nameoffile, sizeof(nameoffile), "data%03d.txt", n);
//   }
//   // Serial.println(n);
//   // Serial.println(nameoffile);
//   //now nameoffile[] contains the name of a file that doesn't exist
//    myFile= SD.open(nameoffile,FILE_READ);
//    myFile.close();
//    myFile= SD.open(nameoffile,FILE_WRITE);
//   // if the file opened okay, write to it:
//   if (myFile) {
//     //Serial2.print("objeeAirlines Autonomous Veichile Test");
    
//     // close the file:
//     myFile.close();
//     //Serial2.println("done.");
//   } else {
//     // if the file didn't open, print an error:
//     //Serial2.println("error opening test.txt");
//   }
// }
