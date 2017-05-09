#include "barometer.h"
#include "../variables.h"


  void samplebarometer(Adafruit_BMP085 &barometer)
  {
  pressure=barometer.readPressure();

  myAltitude=barometer.readAltitude(intialpressure);
  }
