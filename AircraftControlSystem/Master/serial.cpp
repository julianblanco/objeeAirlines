#include "serial.h"
#include "variables.h"
#include "Navigation.h"
//===================================================================
//
//  Program:    Navigation.cpp
//
//  Author:     Julian Blanco
//  Date:     Sep 4 2015
//
//  Description:  Description
//
//===================================================================
 int kp,ki,kd,usekp,useki,usekd;
 int controlchoice=0;
  
// Serial print to com port and xbee to matlab
////////////////////////////////////////////////////////////
void notify_user(HardwareSerial &xbeeSer)
{ 
        xbeeSer.print(rollServoOutput,6);xbeeSer.print(',');
        xbeeSer.print(pitchServoOutput,6);xbeeSer.print(',');
        xbeeSer.print(targetLat,6);xbeeSer.print(',');
        xbeeSer.print(targetLong,6);xbeeSer.print(',');
        xbeeSer.print(targetHeading,1);xbeeSer.print(',');
        xbeeSer.print(trueHeading,1);xbeeSer.print(',');
        xbeeSer.print(GpsSpeed,1);xbeeSer.print(',');
        xbeeSer.print(waypointNumber,1);xbeeSer.print(',');
        xbeeSer.print(distanceToTarget,1);xbeeSer.print(',');
        xbeeSer.print(myAltitude,1);xbeeSer.print(',');
        xbeeSer.print(rollInput,1);xbeeSer.print(',');
        xbeeSer.print(rollTargetValue,1);xbeeSer.print(',');
        xbeeSer.print(pitchSetpoint,1);xbeeSer.print(',');
        xbeeSer.print(pitchInput,1);xbeeSer.print(',');
        xbeeSer.print(leftServoOutput,1);xbeeSer.print(',');
        xbeeSer.print(rightServoOutput,1);xbeeSer.print(',');
        xbeeSer.println(armed,1);
      
      
}


void recvWithStartEndMarkers(HardwareSerial &xbeeSer, HardwareSerial &otherser) {
    static boolean recvInProgress = false;
    
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
    const byte numChars = 74;
    static char receivedChars[numChars];             // IF BROKEN LOOK HERE, array of chars -> string
  //x
 // if (Matlab.available() > 0) {
    if (xbeeSer.available() ) {
        rc = xbeeSer.read();
        //xbeeSer.print(rc);
        //xbeeSer.println("shtisandgiggle");
        if (recvInProgress == true) {
            if (rc != endMarker) {
             // xbeeSer.println(ndx);
                receivedChars[ndx] = rc;
                //xbeeSer.print(receivedChars[ndx]);
                 // xbeeSer.println("test");
                 // xbeeSer.println(receivedChars);
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                    // ndx=0;
                    // rc=NULL;
                    // memset(receivedChars, 0, sizeof(receivedChars));
                    // xbeeSer.println("test1");

                    // xbeeSer.println(receivedChars);

                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                //xbeeSer.println("");
                //xbeeSer.println(receivedChars);
                tokenCreator(receivedChars,sizeof(inputString),xbeeSer);
                                    //xbeeSer.println("test2");
                 
                stringComplete = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
                               //xbeeSer.println("test3");

           // xbeeSer.println(receivedChars);

        }
    }
}


void tokenCreator(char instr[],int strleng,HardwareSerial &xbeeSer){
 char * pch;
  //Serial.println("Splitting string \"%s\" into tokens:\n");

// strtok returns a pointer to a character array, not an index.
// char *s = strtok(receivedBytes,",");
// Serial.println(s);
 //xbeeSer.println(instr);
  pch = strtok (instr,",");
  int index=0;
  while (pch != NULL)
  {

    stringparse(pch,index,xbeeSer);
    pch = strtok (NULL, ",");
    //Serial.println(a);
  index++;
  
  }
}

void stringparse(char buffer[80],int ind,HardwareSerial &xbeeSer)
{
 // xbeeSer.println(buffer);
  static int choice=0;
  //Serial.print(buffer);
  if((strcmp(buffer,"$OA008")==0))choice=1;
  if((strcmp(buffer,"$OA009")==0))choice=2;

  if(choice==1){

    if(ind==1) myLat=atof(buffer);
    if(ind==2) myLong=atof(buffer);
    if(ind==3) myAltitude=atof(buffer);
    if(ind==4) choice=0;
    inputString="";
  }
}