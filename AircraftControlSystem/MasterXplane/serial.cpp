// #include "serial.h"
// #include "variables.h"
// #include "Navigation.h"
// //===================================================================
// //
// //  Program:    Navigation.cpp
// //
// //  Author:     Julian Blanco
// //  Date:     Sep 4 2015
// //
// //  Description:  Description
// //
// //===================================================================
//  int kp,ki,kd,usekp,useki,usekd;
//  int controlchoice=0;
  
// // Serial print to com port and xbee to matlab
// ////////////////////////////////////////////////////////////
// void notify_user(usb_serial_class &xbeeSer)
// { 
//         xbeeSer.print(myLat,6);xbeeSer.print(',');
//         xbeeSer.print(myLong,6);xbeeSer.print(',');
//         xbeeSer.print(targetLat,6);xbeeSer.print(',');
//         xbeeSer.print(targetLong,6);xbeeSer.print(',');
//         xbeeSer.print(targetHeading,1);xbeeSer.print(',');
//         xbeeSer.print(trueHeading,1);xbeeSer.print(',');
//         xbeeSer.print(Gpsheading,1);xbeeSer.print(',');
//         xbeeSer.print(GpsSpeed,1);xbeeSer.print(',');
//         xbeeSer.print(AirSpeed,1);xbeeSer.print(',');
//         xbeeSer.print(waypointNumber,1);xbeeSer.print(',');
//         xbeeSer.print(distanceToTarget,1);xbeeSer.print(',');
//         xbeeSer.print(myAltitude,1);xbeeSer.print(',');
//         xbeeSer.print(rollInput,1);xbeeSer.print(',');
//         xbeeSer.print(pitchInput,1);xbeeSer.print(',');
//         xbeeSer.print(armed,1);xbeeSer.print(',');
//         xbeeSer.print(rollServoOutput);xbeeSer.print(',');
//         xbeeSer.print(pitchServoOutput);xbeeSer.print(',');
//         xbeeSer.print(rollSetpoint);xbeeSer.print(',');
//         xbeeSer.print(pitchSetpoint);xbeeSer.print(',');
//         xbeeSer.print(winddir);xbeeSer.print(',');
//         xbeeSer.print(windmag);xbeeSer.print(',');
//         xbeeSer.print(throtSetpoint);xbeeSer.print(',');
//         xbeeSer.print(pitchAccum,6);xbeeSer.print(',');
//         xbeeSer.println(lastlong,6);
        
// }



// void recvWithStartEndMarkers(usb_serial_class &xbeeSer) {
//     static boolean recvInProgress = false;
    
//     static byte ndx = 0;
//     char startMarker = '<';
//     char endMarker = '>';
//     char rc;
//     const byte numChars = 74;
//     static char receivedChars[numChars];             // IF BROKEN LOOK HERE, array of chars -> string
//   //x
//  // if (Matlab.available() > 0) {
//     if (xbeeSer.available() ) {
//         rc = xbeeSer.read();
//         //xbeeSer.print(rc);
//         //xbeeSer.println("shtisandgiggle");
//         if (recvInProgress == true) {
//             if (rc != endMarker) {
//              // xbeeSer.println(ndx);
//                 receivedChars[ndx] = rc;
//                 //xbeeSer.print(receivedChars[ndx]);
//                  // xbeeSer.println("test");
//                  // xbeeSer.println(receivedChars);
//                 ndx++;
//                 if (ndx >= numChars) {
//                     ndx = numChars - 1;
//                     // ndx=0;
//                     // rc=NULL;
//                     // memset(receivedChars, 0, sizeof(receivedChars));
//                     // xbeeSer.println("test1");

//                     // xbeeSer.println(receivedChars);

//                 }
//             }
//             else {
//                 receivedChars[ndx] = '\0'; // terminate the string
//                 recvInProgress = false;
//                 ndx = 0;
//                 //xbeeSer.println("");
//                 //xbeeSer.println(receivedChars);
//                 tokenCreator(receivedChars,sizeof(inputString),xbeeSer);
//                                     //xbeeSer.println("test2");
                 
//                 stringComplete = true;
//             }
//         }

//         else if (rc == startMarker) {
//             recvInProgress = true;
//                                //xbeeSer.println("test3");

//            // xbeeSer.println(receivedChars);

//         }
//     }
// }


// void tokenCreator(char instr[],int strleng,usb_serial_class &xbeeSer){
//  char * pch;
//   //Serial.println("Splitting string \"%s\" into tokens:\n");

// // strtok returns a pointer to a character array, not an index.
// // char *s = strtok(receivedBytes,",");
// // Serial.println(s);
//  //xbeeSer.println(instr);
//   pch = strtok (instr,",");
//   int index=0;
//   while (pch != NULL)
//   {

//     stringparse(pch,index,xbeeSer);
//     pch = strtok (NULL, ",");
//     //Serial.println(a);
//   index++;
  
//   }
// }


// void stringparse(char buffer[80],int ind,usb_serial_class &xbeeSer)
// {
//   // xbeeSer.println(buffer);
//   static int choice=0;
//   //Serial.print(buffer);
//   if(ind==0){
//   if((strcmp(buffer,"08")==0))choice=1;
//   if((strcmp(buffer,"09")==0))choice=3;
//   if((strcmp(buffer,"10")==0))choice=2;
//   }

//   if(choice==1){

//     if(ind==1) myLat=atof(buffer);
//     if(ind==2) myLong=atof(buffer);
//     // if(ind==3) myAltitude=atof(buffer);
//     if(ind==3) GpsSpeed=atof(buffer);
//     if(ind==4) AirSpeed=atof(buffer);
//     if(ind==5) choice=0;
//     inputString="";
//   }

//     if(choice==2){

//     if(ind==1) pitchInput=atof(buffer);
//     if(ind==2) rollInput=atof(buffer);
//     if(ind==3) trueHeading=atof(buffer);
//     if(ind==4) myAltitude=atof(buffer);
//     if(ind==5) choice=0;
//     inputString="";
//   }
//   if(choice==3){

//     if(ind==1) myLat=atof(buffer);
//     if(ind==2) myLong=atof(buffer);
//     if(ind==3) myAltitude=atof(buffer);
//     if(ind==4) pitchInput=atof(buffer);
//     if(ind==5) rollInput=atof(buffer);
//     if(ind==6) trueHeading=atof(buffer);
//     if(ind==7) GpsSpeed=atof(buffer);
//     if(ind==8) choice=0;
//     inputString="";
//   }
// }


// // void stringChecker(usb_serial_class &xbeeSer)
// // {

  
// //   if (stringComplete) {
// //    // xbeeSer.println(inputString);
// //     // clear the string:

// //     // CheckSum Need to create

// //   //    if (nmea[strlen(nmea)-4] == '*') {
// //   //   uint16_t sum = parseHex(nmea[strlen(nmea)-3]) * 16;
// //   //   sum += parseHex(nmea[strlen(nmea)-2]);
    
// //   //   // check checksum 
// //   //   for (uint8_t i=1; i < (strlen(nmea)-4); i++) {
// //   //     sum ^= nmea[i];
// //   //   }
// //   //   if (sum != 0) {
// //   //     // bad checksum :(
// //   //     //return false;
// //   //   }
// //   // }
// //    // const char * c = str.c_str();
// //  xbeeSer.println("HI JOHNSs");
// //  xbeeSer.println(inputString); 
// //  const char *p = inputString.c_str();
 
// // //Adds Waypoint
// //   if (strstr(p, "$OA001")) {
// //     // found GGA
// //      // get time

// //     // float newwpLat=0;
// //     // float newwpLong=0;
// //     // p = strchr(p, ',')+1;
// //     //  newwpLat = atof(p);
    
// //     // p = strchr(p, ',')+1;
// //     // if (',' != *p)
// //     // {
// //     //  newwpLong = atof(p);
// //     // }
// //     //  wpadd(newwpLat,newwpLong);
// //     // inputString = "";
// //     // stringComplete = false;

// //     xbeeSer.println("HI JOHNS");
// //     p = strchr(p, ',')+1;
// //     myLat = atof(p);
    
// //     p = strchr(p, ',')+1;
// //     if (',' != *p)
// //     {
// //     myLong=atof(p);
// //     }
// //     p = strchr(p, ',')+1;
// //     if (',' != *p)
// //     {
// //     myAltitude=atof(p);
// //     }
// //     inputString = "";
// //     stringComplete = false;
    
// //     return;
// //   }//end $OA001


// // //Prints Waypoints
// //      if (strstr(p, "$OA002")) {
// //     // found GGA
// //      // get time
// //     xbeeSer.print(numOfWaypoints);
// //     for(int x=0;x<=numOfWaypoints;x++)
// //     {   xbeeSer.print(",");xbeeSer.print(wplat[x],6);
// //         xbeeSer.print(",");xbeeSer.print(wplong[x],6);
// //     }
// //     xbeeSer.println("");
// //     inputString = "";
// //     stringComplete = false;
// //     return;
// //   }

// //     if (strstr(p, "$OA003")) {
// //     // found GGA

// //     p = strchr(p, ',')+1;
// //     waypointNumber = atoi(p);
// //     //xbeeSer.print("WP New:");xbeeSer.println(waypointNumber);
// //     inputString = "";
// //     stringComplete = false;
// //     return;
// //   }//end $OA003


// // //heading override
// //     if (strstr(p, "$OA004")) {
// //     // found GGA

// //     p = strchr(p, ',')+1;
// //     waypointNumber = atoi(p);
// //     //xbeeSer.print("WP New:");xbeeSer.println(waypointNumber);
// //     inputString = "";
// //     stringComplete = false;
// //     return;
// //   }//end $OA003

// // //heading override
// //     if (strstr(p, "$OA005")) {
// //     // found GGA

// //     p = strchr(p, ',')+1;
// //     if(armed)armed=0;
// //     else armed=1;
// //     //xbeeSer.print("Armed: "); xbeeSer.println(armed);
// //     inputString = "";
// //     stringComplete = false;
// //     return;
// //   }//end $OA003

// // //heading override
// //     if (strstr(p, "$OA006")) {
// //     // found GGA
    
// //     p = strchr(p, ',')+1;
// //     controlchoice = atoi(p);
    
// //     p = strchr(p, ',')+1;
// //     if (',' != *p)
// //     {
// //     kp=atoi(p);
// //     }
// //     p = strchr(p, ',')+1;
// //     if (',' != *p)
// //     {
// //     ki=atoi(p);
// //     }
// //     p = strchr(p, ',')+1;
// //     if (',' != *p)
// //     {
// //     kd=atoi(p);
// //     }
// //     p = strchr(p, ',')+1;
// //     if (',' != *p)
// //     {
// //     usekp=atoi(p);
// //     }
// //     p = strchr(p, ',')+1;
// //     if (',' != *p)
// //     {
// //     useki=atoi(p);
// //     }

// //     p = strchr(p, ',')+1;
// //     if (',' != *p)
// //     {
// //     usekd=atoi(p);
// //     }
// //     assignpid(controlchoice,kp ,ki,kd,usekd,useki,usekd);
// //     inputString = "";
// //     stringComplete = false;
    
// //     return;
// //   }//end $OA003

// //  if (strstr(p, "$OA007")) {
// //     // found GGA
// //      // get time

// //     for(int x=0;x<=numOfWaypoints;x++)
// //     {   xbeeSer.print(x);xbeeSer.print(" of ");xbeeSer.println(numOfWaypoints);
// //         xbeeSer.print("LAT: ");xbeeSer.print(wplat[x],6);
// //         xbeeSer.print(": LONG: ");xbeeSer.println(wplong[x],6);
// //     }
// //     inputString = "";
// //     stringComplete = false;
// //     return;
// //   }
// //    if (strstr(p, "$OA008")) {
// //     // found GGA
// //     xbeeSer.println("HI JOHNS");
// //     p = strchr(p, ',')+1;
// //     myLat = atof(p);
    
// //     p = strchr(p, ',')+1;
// //     if (',' != *p)
// //     {
// //     myLong=atof(p);
// //     }
// //     p = strchr(p, ',')+1;
// //     if (',' != *p)
// //     {
// //     myAltitude=atof(p);
// //     }
// //     inputString = "";
// //     stringComplete = false;
    
// //     return;

// //   }//end str complete
// //     else
// // {
// //   inputString = "";
// //   stringComplete = false;
// // }


// // return;
// // }
// // }//end fuction

// // void serialEvent(usb_serial_class &xbeeSer,usb_serial_class &xbeeSer2) {
// //   if (xbeeSer.available()) {
// //     // get the new byte:
// //     char inChar = (char)xbeeSer.read();
// //     //xbeeSer2.println(inChar);
// //     // add it to the inputString:
// //     inputString += inChar;
// //     // if the incoming character is a newline, set a flag
// //     // so the main loop can do something about it:
// //     if (inChar == '\n') {
// //       xbeeSer.println("hello");
// //       stringComplete = true;

// //       char *parr = (char*)malloc(sizeof(inputString));
// //       strcpy(parr,inputString.c_str());
// //       //tokenCreator(parr,sizeof(inputString));
// //       free(parr);
// //     }
// //   }
// // }

// // void assignpid(int control,int kp, int ki ,int kd, int usekp,int useki, int usekd)
// // {
// // if(control==1){
// //   rollKp=kp;
// //   rollKi=ki;
// //   rollKd=kd;
// //   userollKp=usekp;
// //   userollKi=useki;
// //   userollKd=usekd;
// // }
// // if(control==2){
// //   pitchKp=kp;
// //   pitchKi=ki;
// //   pitchKd=kd;
// //   usepitchKp=usekp;
// //   usepitchKi=useki;
// //   usepitchKd=usekd;
// // }
// // if(control==3){
// //   yawKp=kp;
// //   yawKi=ki;
// //   yawKd=kd;
// //   useyawKp=usekp;
// //   useyawKi=useki;
// //   useyawKd=usekd;
// // }
// // }
