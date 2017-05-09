
#include "i2cParser.h"

// how long are max NMEA lines to parse?
#define MAXLINELENGTH 120

// we double buffer: read one line in and leave one for the main program
volatile char iline1[MAXLINELENGTH];
volatile char iline2[MAXLINELENGTH];
// our index into filling the current line
volatile uint8_t ilineidx = 0;
// pointers to the double buffers
volatile char *icurrentline;
volatile char *ilastline;
volatile boolean irecvdflag;
volatile boolean iinStandbyMode;


boolean i2cSlave::iparse(char *nmea) {

  if (nmea[strlen(nmea)] == '*') {
    return true;
  }


  if (strstr(nmea, "$GNGYRO")) {
    // found GGA
    char *p = nmea;

    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      calibration = atoi(p);
    }

    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      xdeg = atof(p);
    }

    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      ydeg = atof(p);
    }

    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      zdeg = atof(p);
    }


    return true;
  }

  if (strstr(nmea, "$GNBARO")) {
    // found GGA
    char *p = nmea;

    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      altitude = atof(p);
    }

    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      barometricpressure = atof(p);
    }

    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      sealevel = atof(p);
    }

    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      calibratedaltitude = atof(p);
    }

  }//end baro
}//endparse
  char i2cSlave::iread(void) {
    char c = 0;
    if (paused) return c;
    if (!gpsHwSerial->available()) return c;
    c = gpsHwSerial->read();

    if (c == '\n') {
      icurrentline[ilineidx] = 0;

      if (icurrentline == iline1) {
        icurrentline = iline2;
        ilastline = iline1;
      } else {
        icurrentline = iline1;
        ilastline = iline2;
      }

      ilineidx = 0;
      irecvdflag = true;
    }

    icurrentline[ilineidx++] = c;
    if (ilineidx >= MAXLINELENGTH)ilineidx = MAXLINELENGTH - 1;

    return c;
  }



  // Constructor when using HardwareSerial
  i2cSlave::i2cSlave(HardwareSerial * ser) {
    icommon_init();  // Set everything to common state, then...
    gpsHwSerial = ser; // ...override gpsHwSerial with value passed.
  }

  // Initialization code used by all constructor types
  void i2cSlave::icommon_init(void) {
    gpsHwSerial = NULL; // port pointer in corresponding constructor
    irecvdflag   = false;
    paused      = false;
    ilineidx     = 0;
    icurrentline = iline1;
    ilastline    = iline2;
  }

  void i2cSlave::ibegin(uint16_t baud) {

    gpsHwSerial->begin(baud);
    delay(10);
  }


  boolean i2cSlave::inewNMEAreceived(void) {
    return irecvdflag;
  }

  char *i2cSlave::ilastNMEA(void) {
    irecvdflag = false;
    return (char *)ilastline;
  }

  boolean i2cSlave::iwaitForSentence(const char *wait4me, uint8_t max) {
    char str[20];

    uint8_t i = 0;
    while (i < max) {
      if (inewNMEAreceived()) {
        char *nmea = ilastNMEA();
        strncpy(str, nmea, 20);
        str[19] = 0;
        i++;

        if (strstr(str, wait4me))
          return true;
      }
    }

    return false;
  }
