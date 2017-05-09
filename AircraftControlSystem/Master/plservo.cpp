#include "plservo.h"
//===================================================================
//
//  Program:    Navigation.cpp
//
//  Author:     Julian Blanco
//  Date:     Sep 4 2015
//
//  Description:  Description
//
//===================================================================
void updateServos(Servo &leftServo, Servo &rightServo, Servo &throtServo) {
    // Control pitch based on airspeed error
  if(armed==1)  
  {
  throtServo.writeMicroseconds(1000);
  }
  else
  {
    throtServo.writeMicroseconds(1000);    
  }
  leftServo.writeMicroseconds(leftServoOutput);
  rightServo.writeMicroseconds(rightServoOutput);
}

void ServoInit(Servo &leftServo, Servo &rightServo) {
  // Initialize control values
  leftServo.writeMicroseconds(1500);
  rightServo.writeMicroseconds(1500);
  delay(500);
  leftServo.writeMicroseconds(1900);
  delay(500);
  rightServo.writeMicroseconds(1900);
  leftServo.writeMicroseconds(1500);
  delay(500);
  rightServo.writeMicroseconds(1500);
  leftServo.writeMicroseconds(1100);
  delay(500);
  rightServo.writeMicroseconds(1100);
  delay(500);
  rightServo.writeMicroseconds(1500);
  delay(1000);
}


void ServoWiggle(Servo &leftServo)  {
  // Initialize control values

  for(int i = 0; i <=2;i+=1)
  {
     for (int pos = 1500; pos <= 1950; pos += 5) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      leftServo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(4);                       // waits 15ms for the servo to reach the position
    }
    for (int pos = 1950; pos >= 1150; pos -= 5) { // goes from 180 degrees to 0 degrees
      leftServo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(4);                       // waits 15ms for the servo to reach the position
    }
     for (int pos = 1150; pos <= 1500; pos += 5) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      leftServo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(4);                       // waits 15ms for the servo to reach the position
    }
  }
}




void CenterAll(Servo &leftServo, Servo &rightServo,Servo &throtServo)  {
  // Center all servos
  leftServo.writeMicroseconds(1500);
  rightServo.writeMicroseconds(1500);
  throtServo.writeMicroseconds(1100);
}

void servoBegin(Servo &leftServo, Servo &rightServo, Servo &throtServo) {
  leftServo.attach(3);
  rightServo.attach(4);
  throtServo.attach(5);
  leftServo.writeMicroseconds(1500);
  rightServo.writeMicroseconds(1500);
  throtServo.writeMicroseconds(1050);
}




