
#include "serial.h"
#include "variables.h"
#include "Navigation.h"

 int kp,ki,kd,usekp,useki,usekd;
 int controlchoice=0;
  
// Serial print to com port and xbee to matlab
////////////////////////////////////////////////////////////
void notify_user(HardwareSerial &xbeeSer)
{ static int x=0;
  x++;
  if(x>10){
        xbeeSer.print(myLat,6);xbeeSer.print(',');
        xbeeSer.print(myLong,6);xbeeSer.print(',');
        xbeeSer.print(targetLat,6);xbeeSer.print(',');
        xbeeSer.print(targetLong,6);xbeeSer.print(',');
        xbeeSer.print(targetHeading,4);xbeeSer.print(',');
        xbeeSer.print(trueHeading,4);xbeeSer.print(',');
        xbeeSer.print(GpsSpeed);xbeeSer.print(',');
        xbeeSer.print(waypointNumber);xbeeSer.print(',');
        xbeeSer.print(distanceToTarget,4);xbeeSer.print(',');
        xbeeSer.print(myAltitude,4);xbeeSer.print(',');
        xbeeSer.print(rollInput);xbeeSer.print(',');
        xbeeSer.print(pitchInput);xbeeSer.print(',');
        xbeeSer.print(armed);xbeeSer.print(',');
        xbeeSer.println(pressure,4);//xbeeSer.print(',');
        x=0;
      }
}


void stringChecker(HardwareSerial &xbeeSer)
{
  if (stringComplete) {
    xbeeSer.println(inputString);
    // clear the string:

    // CheckSum Need to create

  //    if (nmea[strlen(nmea)-4] == '*') {
  //   uint16_t sum = parseHex(nmea[strlen(nmea)-3]) * 16;
  //   sum += parseHex(nmea[strlen(nmea)-2]);
    
  //   // check checksum 
  //   for (uint8_t i=1; i < (strlen(nmea)-4); i++) {
  //     sum ^= nmea[i];
  //   }
  //   if (sum != 0) {
  //     // bad checksum :(
  //     //return false;
  //   }
  // }
   // const char * c = str.c_str();

 const char *p = inputString.c_str();
 
//Adds Waypoint
  if (strstr(p, "$OA001")) {
    // found GGA
     // get time

    float newwpLat=0;
    float newwpLong=0;
    p = strchr(p, ',')+1;
     newwpLat = atof(p);
    
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
     newwpLong = atof(p);
    }
     wpadd(newwpLat,newwpLong);
    inputString = "";
    stringComplete = false;
    return;
  }//end $OA001


//Prints Waypoints
     if (strstr(p, "$OA002")) {
    // found GGA
     // get time

    for(int x=0;x<=numOfWaypoints;x++)
    {   xbeeSer.print(x);xbeeSer.print(" of ");xbeeSer.println(numOfWaypoints);
        xbeeSer.print("LAT: ");xbeeSer.print(wplat[x]);
        xbeeSer.print(": LONG: ");xbeeSer.println(wplong[x]);
    }
    inputString = "";
    stringComplete = false;
    return;
  }

    if (strstr(p, "$OA003")) {
    // found GGA

    p = strchr(p, ',')+1;
    waypointNumber = atoi(p);
    //xbeeSer.print("WP New:");xbeeSer.println(waypointNumber);
    inputString = "";
    stringComplete = false;
    return;
  }//end $OA003


//heading override
    if (strstr(p, "$OA004")) {
    // found GGA

    p = strchr(p, ',')+1;
    waypointNumber = atoi(p);
    //xbeeSer.print("WP New:");xbeeSer.println(waypointNumber);
    inputString = "";
    stringComplete = false;
    return;
  }//end $OA003

//heading override
    if (strstr(p, "$OA005")) {
    // found GGA

    p = strchr(p, ',')+1;
    armed = atoi(p);
    //xbeeSer.print("Armed: "); xbeeSer.println(armed);
    inputString = "";
    stringComplete = false;
    return;
  }//end $OA003

//heading override
    if (strstr(p, "$OA006")) {
    // found GGA
    
    p = strchr(p, ',')+1;
    controlchoice = atoi(p);
    
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
    kp=atoi(p);
    }
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
    ki=atoi(p);
    }
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
    kd=atoi(p);
    }
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
    usekp=atoi(p);
    }
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
    useki=atoi(p);
    }

    p = strchr(p, ',')+1;
    if (',' != *p)
    {
    usekd=atoi(p);
    }
    assignpid(controlchoice,kp ,ki,kd,usekd,useki,usekd);
    inputString = "";
    stringComplete = false;
    
    return;
  }//end $OA003



  }//end str complete
}//end fuction

void serialEvent(HardwareSerial &xbeeSer) {
  if (xbeeSer.available()) {
    // get the new byte:
    char inChar = (char)xbeeSer.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
      if(stringComplete)Serial.println(inputString);
    }
  }
}

void assignpid(int control,int kp, int ki ,int kd, int usekp,int useki, int usekd)
{
if(control==1){
  rollKp=kp;
  rollKi=ki;
  rollKd=kd;
  userollKp=usekp;
  userollKi=useki;
  userollKd=usekd;
}
if(control==2){
  pitchKp=kp;
  pitchKi=ki;
  pitchKd=kd;
  usepitchKp=usekp;
  usepitchKi=useki;
  usepitchKd=usekd;
}
if(control==3){
  yawKp=kp;
  yawKi=ki;
  yawKd=kd;
  useyawKp=usekp;
  useyawKi=useki;
  useyawKd=usekd;
}
}