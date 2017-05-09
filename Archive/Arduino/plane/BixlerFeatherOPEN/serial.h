#ifndef _SERIAL_H_
#define _SERIAL_H_

#include "Arduino.h"
void notify_user(HardwareSerial &xbeeSer);
void stringChecker(HardwareSerial &xbeeSer);
void serialEvent(HardwareSerial &xbeeSer);
void assignpid(int control,int kp, int ki ,int kd, int usekp,int useki, int usekd);
#endif
