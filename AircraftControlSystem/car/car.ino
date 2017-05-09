#include <SD.h>
#include <Servo.h>
#include "files/Adafruit_BNO055.h"
Servo steer;
Servo throttle;

Adafruit_BNO055 bno = Adafruit_BNO055();

#include <Arduino.h>
#include "wiring_private.h" // pinPeripheral() function

uint32_t counter=0;
#define delayTime 30
//#include "variables.h"
//Football trianlge
#include <stdint.h>
#include "Arduino.h"
//===================================================================
//
//	Program:		Navigation.cpp
//
//	Author:			Julian Blanco
//	Date:			Sep 4 2015
//
//	Description:	Description
//
//===================================================================
float heading=0;
float pitchSetpoint =-31;


float pitchdot = 0;

float pitchInput, pitchServoOutput=0;
float yawSetpoint, yawInput, yawServoOutput=0;

//Specify the links and initial tuning parameters
float pitchKp = 65, pitchKi = .2, pitchKd = 1200;

//Specify the links and initial tuning parameters
float yawKp = 8, yawKi = .1, yawKd = .5;

float pitchAccum=0;
float yawAccum=0;


float pitchErrorOld = 0;
float yawErrorOld = 0;

float dpitchResponse =0;
float dyawResponse =0;

float ipitchResponse =0;
float iyawResponse =0;

float ppitchResponse =0;
float pyawResponse =0;

float pitchResponse =0;
float yawResponse =0;

float derivativeSetpoint = 0;

int yawOffset=0;

float targetHeading =0 ;

int usepitchKp;
int usepitchKi;
int usepitchKd;
int useyawKp;
int useyawKi;
int useyawKd;
float calibration=0;

float yawError=0;
float pitchError=0;
// Loop variables
uint16_t dt;  // Stores time in micros for gyro integration
uint32_t start;
uint32_t finish;
int error;                    // Error code returned when MPU6050 is read
int temp;

float rollInput=0;


float throtSetpoint=1500;
//float desiredthrotSetpoint=1500;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
int armed =0;


float throttleSetpoint=0;

void setup(void) 
{

  Serial.begin(9600);

  servoBegin(steer,throttle);
  delay(1000);
  SDinit();
  Wire.begin();  // Initialize I2C bus
  if(!bno.begin())while(1){};
  bno.setExtCrystalUse(true);
  SampleGyro(bno);

  // while(calibration != 3)
  // {
  //    delay(500);
   //   SampleGyro(bno);
   //}
   // Initialize servos
   imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
   yawOffset= euler.x();
   throttle.writeMicroseconds(1500);
   steer.writeMicroseconds(1500);
   delay(1000);

}//end setup
 uint32_t tiempo =0;

void loop(void)
{
  tiempo=millis();
  SampleGyro(bno);
  ControlResponse(targetHeading, pitchSetpoint, yawInput, pitchInput);
  if (pitchInput<-75) {throttleSetpoint =1500;steer.writeMicroseconds(1500);}
  WriteServo(throttle,steer);
  //if ((counter%100)==0) Serialprint();
  counter++;
  sdlog();
  //if((counter%100)==0)notifyuser();
  while(millis()<(tiempo+delayTime)) {
  }

}//end main


void notifyuser()
{
  Serial.print(throttleSetpoint);Serial.print(",");
  Serial.println(pitchInput);

}
// 2 surface plane
void ControlResponse(float &yawTargetValue, float &pitchTargetValue, float yawIn, float pitchIn) 
{
  


  // Calculate heading error
   yawError =  angular_diff( yawTargetValue,yawIn);
  //Calculate pitch error
  pitchError = pitchTargetValue - pitchIn;

  //Calculate ServoOutputs based on PID control
   
  throttleSetpoint= controlPID(pitchError,pitchKp,pitchKi,pitchKd,1,1,1,200,1500,pitchAccum, pitchErrorOld);
  throttleSetpoint=saturate(throttleSetpoint,2000,1000);   // this is actually the right servo command in microseconds 1000-2000

  //Treating plane as bank and yaw no rudder
  //yawServoOutput=  ((cos(abs(rollIn)*3.1415/180)*(yawServoOutput-1500)) + sin(abs(rollIn)*3.1415/180)*(pitchServoOutput-1550))+1500;
}


float controlPID(float ErrorCurrent, float Kp, float Ki, float Kd, int useKP, int useKI, int useKD, float satuarationpoint, float offset, float &Integralresponse, float &ErrorOld) 
{
  //**************************** Proportional***********************************************
  //Calculate proportional response
  ppitchResponse = ErrorCurrent * Kp;

  //**************************** Integral***********************************************
  //Calculate Integral response and add to accumulator: THINK ABOUT HOW TO RESET THE INTEGRAL RESPONSE
  pitchAccum = pitchAccum+(ErrorCurrent*Ki );
  ipitchResponse=pitchAccum;
  if (ipitchResponse > satuarationpoint) ipitchResponse = satuarationpoint;
 if (ipitchResponse < -1*satuarationpoint) ipitchResponse = -1*satuarationpoint;


  //**************************** Derivative***********************************************

  // time elapsed per clock cycle; it works, but should be 100ms not 10. NEED TO HOOK UP TO OSCOPE
  static float oldtime=0;
  //Calculate dx/dt
  float timePID =millis()-oldtime;
  float derivativeterm =(pitchErrorOld - ErrorCurrent)/timePID;
  oldtime=timePID;
  //pass current variable to error1 and overwrite previous value
  pitchErrorOld = ErrorCurrent;
  //Calculate derivative Response
  dpitchResponse = (derivativeSetpoint-derivativeterm)*Kd;      // tends towards derivativeSetpoint
  //**************************** Total***********************************************
  //Calculate total Response
//float response =  derivativeResponse+1500;// + ipitchResponse*useKI +derivativeResponse*useKD+offset;

  float response = ppitchResponse*useKP+ ipitchResponse*useKI + offset + dpitchResponse*useKD;// + ipitchResponse*useKI +derivativeResponse*useKD+offset;
  return response;
}

void WriteServo(Servo &throtServo,Servo &steerServo) {
  throtServo.writeMicroseconds(throttleSetpoint);
  steerServo.writeMicroseconds(throttleSetpoint);
}

float saturate(float input,float upperbound,float lowerbound)
{
  if(input>upperbound) input=upperbound;
  if(input<lowerbound) input=lowerbound;
  return input;
}




void servoBegin(Servo &yawServo, Servo &throtServo) {
  
  yawServo.attach(3);
  throtServo.attach(9);
  yawServo.writeMicroseconds(1500);
  throtServo.writeMicroseconds(1500);
}


float throttleControl(float desired){
  static float Setpoint=0;

  if (desired > Setpoint) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    Setpoint+=1;
  }
   if (desired < Setpoint) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    Setpoint-=1;
  }

  return Setpoint;
}





void SampleGyro(Adafruit_BNO055 &gyroIMU){
    /* Display the floating  point data */
    imu::Vector<3> euler = gyroIMU.getVector(Adafruit_BNO055::VECTOR_EULER);
    yawInput=euler.x()-yawOffset;

    if (yawInput >= 360)
    {
          yawInput -= 360;
        }
    else 
    {if (yawInput < 0)yawInput += 360;
    }
    pitchInput=euler.y();
    //rollInput=fmod((euler.z()+(360+90)), 360)-180;
    rollInput=euler.z();

    imu::Vector<3> gyrodot = gyroIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    pitchdot=gyrodot.y();



    uint8_t system, gyro, accel, mag = 0;
    gyroIMU.getCalibration(&system, &gyro, &accel, &mag);
    calibration=system;



}

float SampleGyroX(Adafruit_BNO055 &gyroIMU){
    /* Display the floating  point data */
    imu::Vector<3> euler = gyroIMU.getVector(Adafruit_BNO055::VECTOR_EULER);
    yawInput=euler.x()-yawOffset;

    if (yawInput >= 360)
    {
          yawInput -= 360;
        }
    else 
    {if (yawInput < 0)yawInput += 360;
    }
}

// Find the shortest signed angular difference between a target and source angle
float angular_diff(float target_angle, float source_angle)
{
        // Find simple difference
        float diff = target_angle - source_angle;
        if (diff > 180)
                diff -= 360;
        else if (diff < -180)
                diff += 360;

        return(diff);
}



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
        myFile.print(pitchInput,1);myFile.print(',');
        myFile.print(ppitchResponse,1);myFile.print(',');
        myFile.print(ipitchResponse,1);myFile.print(',');
        myFile.print(dpitchResponse,1);myFile.print(',');
        myFile.print(throttleSetpoint,1);myFile.print(',');
        myFile.print(tiempo,1);myFile.print(',');
        myFile.println(pitchdot,1);

      }
       myFile.close();
}


void SDinit()
{
  if (!SD.begin(10)) {
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
    //myFile.println("latitude,longitude");
    // close the file:
    myFile.close();
    //Serial2.println("done.");
  } else {
    // if the file didn't open, print an error:
    //Serial2.println("error opening test.txt");
  }
}