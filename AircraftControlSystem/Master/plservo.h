#ifndef _PLSERVO_H_
#define _PLSERVO_H_

#include "files/ServoM.h"
#include "files/ServoTimerM.h"
#include "variables.h"
#include <Arduino.h>
void updateServos(Servo &leftServo, Servo &rightServo , Servo &throtServo);
void ServoInit(Servo &leftServo, Servo &rightServo);
void ServoWiggle(Servo &leftServo);
void CenterAll(Servo &leftServo, Servo &rightServo, Servo &throtServo);
void servoBegin(Servo &leftServo, Servo &rightServo, Servo &throtServo);
#endif