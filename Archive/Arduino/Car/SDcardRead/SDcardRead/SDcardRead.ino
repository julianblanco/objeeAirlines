/*
  SD card datalogger

 This example shows how to log data from three analog sensors
 to an SD card using the SD library.

 The circuit:
 * analog sensors on analog ins 0, 1, and 2
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4

 created  24 Nov 2010
 modified 9 Apr 2012
 by Tom Igoe

 This example code is in the public domain.

 */

#include <SPI.h>
#include <SD.h>

const int chipSelect = 10;
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void setup() {
  // Open serial communications and wait for port to open:
  pinMode(6, OUTPUT);
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  while (!SD.begin(chipSelect)) {
    digitalWrite(6, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);              // wait for a second
  digitalWrite(6, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);     
  }
  Serial.println("card initialized.");

  inputString.reserve(200);
}

void loop() {
  // make a string for assembling the data to log:
  

  serialEvent(); //call the function
  // print the string when a newline arrives:
  if (stringComplete) {
    Serial.println(inputString);
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    File dataFile = SD.open("datalog.txt", FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(inputString);
      dataFile.close();
      // print to the serial port too:
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening datalog.txt");
    }

    // clear the string:
    inputString = "";
    stringComplete = false;
  }


  // // read three sensors and append to the string:
  // for (int analogPin = 0; analogPin < 3; analogPin++) {
  //   int sensor = analogRead(analogPin);
  //   dataString += String(sensor);
  //   if (analogPin < 2) {
  //     dataString += ",";
  //   }
  // }


}

//Interrupt Service Routine
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}








