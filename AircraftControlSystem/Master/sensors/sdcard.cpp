#include "sdcard.h"
// File myFile;
// char nameoffile[16];
// char nameoffile1[16];
// void sdlog()
// {
//      myFile = SD.open(nameoffile, FILE_WRITE);
//      if (myFile) {
//         myFile.print(myLat,6);myFile.print(',');
//         myFile.print(myLong,6);myFile.print(',');
//         myFile.print(targetLat,6);myFile.print(',');
//         myFile.print(targetLong,6);myFile.print(',');
//         myFile.print(targetHeading,4);myFile.print(',');
//         myFile.print(Gpsheading,4);myFile.print(',');
//         //myFile.print(currentEuler,4);myFile.print(',');
//         myFile.print(trueHeading,4);myFile.print(',');
//         myFile.print(headingError,4);myFile.print(',');
//         myFile.print(GpsSpeed);myFile.print(',');
//         myFile.print(waypointNumber);myFile.print(',');
//         myFile.print(distanceToTarget,4);myFile.print(',');
//         myFile.print(myAltitude,4);myFile.print(',');
//         myFile.print(yawInput);myFile.print(',');
//         myFile.print(rollInput);myFile.print(',');
//         myFile.print(pitchInput);myFile.print(',');
//         myFile.print(calibration);myFile.print(',');
//         myFile.print(yawServoOutput);myFile.print(',');
//         myFile.print(rollServoOutput);myFile.print(',');
//         myFile.print(pitchServoOutput);myFile.print(',');
//         myFile.print(pitchServoOutput);myFile.print(',');
//         myFile.print(myAltitude,4);myFile.print(',');
//         myFile.print(intialAltitude,4);myFile.print(',');
//         myFile.print(intialpressure,4);myFile.print(',');
//         myFile.println(pressure,4);//myFile.print(',');
       
//        // myFile.print(loopspeed);myFile.print(',');
//       }
//        myFile.close();
// }


// void SDinit()
// {
//   if (!SD.begin(4)) {
//    // xbee.println("initialization failed!");
//     return;
//   }
//   //xbee.println("initialization done.");

//   // open the file. note that only one file can be open at a time,
//   // so you have to close this one before opening another.
//    // make it long enough to hold your longest file name, plus a null terminator
//   int n = 0;
//   snprintf(nameoffile, sizeof(nameoffile), "data%03d.txt", n); // includes a three-digit sequence number in the file name
//   while(SD.exists(nameoffile)) {
//     n++;
//     snprintf(nameoffile, sizeof(nameoffile), "data%03d.txt", n);
//   }
//   Serial.println(n);
//   Serial.println(nameoffile);
//   //now nameoffile[] contains the name of a file that doesn't exist
//    myFile= SD.open(nameoffile,FILE_READ);
//    myFile.close();
//    myFile= SD.open(nameoffile,FILE_WRITE);
//   // if the file opened okay, write to it:
//   if (myFile) {
//     //xbee.print("objeeAirlines Autonomous Veichile Test");
//     myFile.println("objeeAirlines Autonomous Vehicle Test");
//     // close the file:
//     myFile.close();
//     //xbee.println("done.");
//   } else {
//     // if the file didn't open, print an error:
//     //xbee.println("error opening test.txt");
//   }
// }

// void sdlogCONT()
// {
//      myFile = SD.open(nameoffile1, FILE_WRITE);
//      if (myFile) {
//         myFile.print(myLat,6);myFile.print(',');
//         myFile.print(myLong,6);myFile.print(',');
//         myFile.print(targetLat,6);myFile.print(',');
//         myFile.print(targetLong,6);myFile.print(',');
//         myFile.print(targetHeading,4);myFile.print(',');
//         myFile.print(Gpsheading,4);myFile.print(',');
//         //myFile.print(currentEuler,4);myFile.print(',');
//         myFile.print(trueHeading,4);myFile.print(',');
//         myFile.print(headingError,4);myFile.print(',');
//         myFile.print(GpsSpeed);myFile.print(',');
//         myFile.print(waypointNumber);myFile.print(',');
//         myFile.print(distanceToTarget,4);myFile.print(',');
//         myFile.print(myAltitude,4);myFile.print(',');
//         myFile.print(yawInput);myFile.print(',');
//         myFile.print(rollInput);myFile.print(',');
//         myFile.print(pitchInput);myFile.print(',');
//         myFile.print(calibration);myFile.print(',');
//         myFile.print(yawServoOutput);myFile.print(',');
//         myFile.print(rollServoOutput);myFile.print(',');
//         myFile.print(pitchServoOutput);myFile.print(',');
//         myFile.print(pitchServoOutput);myFile.print(',');
//         myFile.print(myAltitude,4);myFile.print(',');
//         myFile.print(intialAltitude,4);myFile.print(',');
//         myFile.print(intialpressure,4);myFile.print(',');
//         myFile.println(pressure,4);//myFile.print(',');
       
//        // myFile.print(loopspeed);myFile.print(',');
//       }
//        myFile.close();
// }


// void SDinitCONT()
// {
//   if (!SD.begin(4)) {
//    // xbee.println("initialization failed!");
//     return;
//   }
//   //xbee.println("initialization done.");

//   // open the file. note that only one file can be open at a time,
//   // so you have to close this one before opening another.
//    // make it long enough to hold your longest file name, plus a null terminator
//   int n = 0;
//   snprintf(nameoffile1, sizeof(nameoffile1), "controltest%03d.txt", n); // includes a three-digit sequence number in the file name
//   while(SD.exists(nameoffile1)) {
//     n++;
//     snprintf(nameoffile1, sizeof(nameoffile1), "controltest%03d.txt", n);
//   }
//   Serial.println(n);
//   Serial.println(nameoffile1);
//   //now nameoffile[] contains the name of a file that doesn't exist
//    myFile= SD.open(nameoffile1,FILE_READ);
//    myFile.close();
//    myFile= SD.open(nameoffile1,FILE_WRITE);
//   // if the file opened okay, write to it:
//   if (myFile) {
//     //xbee.print("objeeAirlines Autonomous Veichile Test");
//     //myFile.println("objeeAirlines Autonomous Vehicle Test");
//     // close the file:
//     myFile.close();
//     //xbee.println("done.");
//   } else {
//     // if the file didn't open, print an error:
//     //xbee.println("error opening test.txt");
//   }
// }