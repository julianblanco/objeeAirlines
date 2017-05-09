
#include <Wire.h>

int Address = 2;  //This slave is address number 2
int d =0;
int i =0;

    float Gpsheading; //my gps true heading
    float targetHeading;//my calculted desired heading
    float currentLat;
    float currentLong;
    float targetLat;
    float targetLong;
    float SOG;
    float distanceToTarget;
uint8_t fix;
uint8_t satilites;

uint8_t waypointNumber;




void setup()
{
  Serial.begin(9600);
  Wire.begin(Address);
  Wire.onRequest(requestEvent); // register event
}

void loop()
{ 
  d++;
  Gpsheading = 359.34  ;  //your code here
  targetHeading = 284.2;
  currentLat =-50.22;
  currentLong =73.3333;
  targetLat=29.123;
  targetLong=34.234;
  SOG=3;
 distanceToTarget=10.34;
  delay(1000);
  if(d>5)
   fix=1;
   if(d>10)
    distanceToTarget=1.234;
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent()
{
  struct {
    uint8_t fix;
    uint8_t satilites;
    uint8_t waypointNumber;
    
    float distanceToTarget;
    float Gpsheading; //my gps true heading
    float targetHeading;//my calculted desired heading
    float currentLat;
    float currentLong;
    float targetLat;
    float targetLong;
    
    
  } data = {fix,satilites,waypointNumber,Gpsheading,targetHeading, distanceToTarget,currentLat,
             currentLong, targetLat, targetLong,};
  Wire.write((unsigned char*)&data, sizeof(data));
}
