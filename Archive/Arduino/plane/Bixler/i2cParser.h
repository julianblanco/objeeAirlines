/***********************************
This is the Adafruit GPS library - the ultimate GPS library
for the ultimate GPS module!

Tested and works great with the Adafruit Ultimate GPS module
using MTK33x9 chipset
    ------> http://www.adafruit.com/products/746
Pick one up today at the Adafruit electronics shop 
and help support open source hardware & software! -ada

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.  
BSD license, check license.txt for more information
All text above must be included in any redistribution
****************************************/
// Fllybob added lines 34,35 and 40,41 to add 100mHz logging capability 

#ifndef _I2CPARSER_H
#define _I2CPARSER_H


  

// how long to wait when we're looking for a response
#define MAXWAITSENTENCE 5

#if ARDUINO >= 100
 #include "Arduino.h"

#else
 #include "WProgram.h"
 #include "NewSoftSerial.h"
#endif


class i2cSlave {
 public:
  void ibegin(uint16_t baud); 


  i2cSlave(HardwareSerial *ser); // Constructor when using HardwareSerial

  char *ilastNMEA(void);
  boolean inewNMEAreceived();
  void icommon_init(void);

  void isendCommand(const char *);
  
  void ipause(boolean b);

  boolean iparseNMEA(char *response);
  uint8_t iparseHex(char c);

  char iread(void);
  boolean iparse(char *);
  void iinterruptReads(boolean r);

  boolean iwakeup(void);
  boolean istandby(void);

  uint8_t hour, minute, seconds, year, month, day;
  uint16_t milliseconds;
  // Floating point latitude and longitude value in degrees.
  float latitude, longitude;
  // Fixed point latitude and longitude value with degrees stored in units of 1/100000 degrees,
  // and minutes stored in units of 1/100000 degrees.  See pull #13 for more details:
  //   https://github.com/adafruit/Adafruit-GPS-Library/pull/13
  float calibration;
  float xdeg;
  float ydeg;
  float zdeg;
  float  altitude;
  float barometricpressure;
  float sealevel;
  float calibratedaltitude;
 

  boolean iwaitForSentence(const char *wait, uint8_t max = MAXWAITSENTENCE);


 private:
  boolean paused;
  
  uint8_t parseResponse(char *response);

  HardwareSerial *gpsHwSerial;
};


#endif
