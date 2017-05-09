#ifndef _PLSERVO_H_
#define _PLSERVO_H_

#include "files/ServoM.h"
#include "files/ServoTimerM.h"
#include "variables.h"
#include <Arduino.h>
void UpdateServos(Servo &rollServo, Servo &pitchServo,Servo &yawServo);
void ServoInit(Servo &rollServo, Servo &pitchServo,Servo &yawServo);
void ServoWiggle(Servo &rollServo, Servo &pitchServo,Servo &yawServo);
void CenterAll(Servo &rollServo, Servo &pitchServo,Servo &yawServo);
void servoBegin(Servo &rollServo, Servo &pitchServo,Servo &yawServo);
#endif