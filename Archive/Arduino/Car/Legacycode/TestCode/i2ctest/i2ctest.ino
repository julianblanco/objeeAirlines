
#include <Wire.h>

volatile byte* INPUT1FloatPtr;
volatile byte* INPUT2FloatPtr;
int Address = 2;  //This slave is address number 2
float heading=359.34;
int i =0;
float targetHeading=284.2;
uint8_t fix=0;
int d =0;

void setup()
{
  Serial.begin(9600);
  Wire.begin(Address);
  Wire.onRequest(requestEvent); // register event
}

void loop()
{ 
  d++;
  heading = 359.34  ;  //your code here
  targetHeading = 284.2;
  delay(1000);
  if(d>15)
  fix=1;
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent()
{
  struct {
    uint8_t fix;
    float heading;
    float targetHeading;
  } data = {fix, heading, targetHeading};
  Wire.write((unsigned char*)&data, sizeof(data));
//  float fl = 3.14;
//  Wire.write((uint8_t*)&fl + i, 1);
//  i = (i+1)%4;
//
//  float pi = 3.14159;
//  Serial.println("mark.");
//  Wire.write((uint8_t*)&pi, 4);
//  return;
//  if((i/4)==0)
//  {
//  Wire.write((uint8_t*)&heading + (i%4),1);
//  i++;
//  }
//  else if((i/4)==1)
//  {
//    Wire.write((uint8_t*)&fix + (i%4), 1);
//  i++;
//  }
//  else
//  {
//  Wire.write((uint8_t*)&targetHeading + (i%4), 1);
//  i = (i+1)%12;
// //i=0;
//  }
}
