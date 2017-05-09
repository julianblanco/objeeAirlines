#ifndef _GYRO_H_
#define _GYRO_H_
#include "../files/Adafruit_BNO055.h"
#include "../variables.h"
void SampleGyro(Adafruit_BNO055 &gyroIMU);

float SampleGyroX(Adafruit_BNO055 &gyroIMU);
#endif