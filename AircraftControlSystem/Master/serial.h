#ifndef _SERIAL_H_
#define _SERIAL_H_

#include "Arduino.h"
void notify_user(HardwareSerial &xbeeSer);
//void stringChecker(HardwareSerial &xbeeSer);
//void serialEvent(HardwareSerial &xbeeSer,HardwareSerial &xbeeSer2);
//void assignpid(int control,int kp, int ki ,int kd, int usekp,int useki, int usekd);

void tokenCreator(char instr[],int strleng,HardwareSerial &xbeeSer);
void stringparse(char buffer[80],int ind,HardwareSerial &xbeeSer);
void recvWithStartEndMarkers(HardwareSerial &xbeeSer, HardwareSerial &otherser);
#endif
