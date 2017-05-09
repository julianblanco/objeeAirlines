#include "barometer.h"
#include "../variables.h"


  void samplebarometer(Adafruit_BMP085 &barometer)
  {
  //pressure=barometer.readPressure();
  myAltitude=barometer.readAltitude(intialpressure);
  }

   void samplebarometer(LPS &barometer)
  {
   pressure = barometer.readPressureInchesHg();
   myAltitude = barometer.pressureToAltitudeFeet(pressure,intialpressure);
  }

  float averagebarmoeter(LPS &barometer)
  {
  	float sum =0;
  	for(int idx = 0 ;idx<100;idx++)
  	{
  		sum = sum + barometer.readPressureInchesHg();
  		delay(200);
  	}
  	sum = sum/100;
  	return sum;
  }

  float  averagebarmoeter(Adafruit_BMP085 &barometer)
  {
  	float sum =0;
  	for(int idx = 0 ;idx<100;idx++)
  	{
  		sum = sum + barometer.readPressure();
  		delay(200);
  	}
  	sum = sum/100;
  	return sum;
  }