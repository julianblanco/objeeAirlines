#include "plservo.h"

void UpdateServos(Servo &rollServo, Servo &pitchServo,Servo &yawServo) {
     
  rollServo.writeMicroseconds(rollServoOutput);
  pitchServo.writeMicroseconds(pitchServoOutput);
  yawServo.writeMicroseconds(yawServoOutput);
 
}

void ServoInit(Servo &rollServo, Servo &pitchServo,Servo &yawServo) {
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


void ServoWiggle(Servo &rollServo, Servo &pitchServo,Servo &yawServo)  {
  // Initialize control values

  for(int i = 0; i <=2;i+=1)
  {
     for (int pos = 1500; pos <= 1950; pos += 5) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      rollServo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(4);                       // waits 15ms for the servo to reach the position
    }
    for (int pos = 1950; pos >= 1150; pos -= 5) { // goes from 180 degrees to 0 degrees
      rollServo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(4);                       // waits 15ms for the servo to reach the position
    }
     for (int pos = 1150; pos <= 1500; pos += 5) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      rollServo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(4);                       // waits 15ms for the servo to reach the position
    }
  }
}




void CenterAll(Servo &rollServo, Servo &pitchServo,Servo &yawServo)  {
  // Center all servos
  rollServo.writeMicroseconds(1500);
  pitchServo.writeMicroseconds(1500);
  yawServo.writeMicroseconds(1500);
}

void servoBegin(Servo &rollServo, Servo &pitchServo,Servo &yawServo) {
  rollServo.attach(9);
  yawServo.attach(10);
  pitchServo.attach(11);
  rollServo.writeMicroseconds(1500);
  pitchServo.writeMicroseconds(1500);
  yawServo.writeMicroseconds(1500);
}

