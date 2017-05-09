#include <Servo.h>  //Used to control the Pan/Tilt Servos


//These are the objects for each servo.
Servo servo, motor;

//This is a character that will hold data from the Serial port.
char serialChar=0;
int i=90;
int j=0;
int value=0;
int motorspeed =0;
const int numReadings = 5;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

void setup(){
   servo.attach(10);   //The Pan servo is attached to pin 6.
  motor.writeMicroseconds(1500);  //Initially put the servos both
  servo.write(90);      //at 90 degress
  delay(1500);
  Serial.begin(115200);  //Set up a serial connection for 9600 bps.

for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
}

void loop(){







  while(Serial.available() <=0);  //Wait for the second command byte from the serial port.
 {
  
  i=Serial.parseInt();  //Set the tilt servo position to the value of the second command byte received on the serial port


if(i<1000)
{
Serial.println(i);
 if(i>0)
  {
    value=i;
  }

  if(value>160)
  value=160;
  
  if (i<20)
  value=20;
 

// subtract the first reading:
 
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = value;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;
  // send it to the computer as ASCII digits





  Serial.print("Servo: ");
  
  if(average>150)
  average=150;
  
  if (average<30)
  average=30;

  average=map(average,20,160,160,20);
}
else
{

motorspeed = i;

if(motorspeed<1500)
  motorspeed =1500;
if(motorspeed>1800)
  motorspeed=1800;
}
  servo.write(average);
  motor.writeMicroseconds(motorspeed);
 // Serial.println(average);
 }

}


