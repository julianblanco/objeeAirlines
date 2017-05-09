#ifndef _BAROMETER_H_
#define _BAROMETER_H_s

#include "../files/Adafruit_BMP085.h"
#include "../files/LPS.h"
void samplebarometer(Adafruit_BMP085 &barometer);
float averagebarmoeter(Adafruit_BMP085 &barometer);
void samplebarometer(LPS &barometer);
float averagebarmoeter(LPS &barometer);
#endif