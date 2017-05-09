
#include <Wire.h>

#include <Adafruit_GPS.h>

#include <SoftwareSerial.h>

#include <waypointClass.h>    // custom class to manaage GPS waypoints
#include <math.h>  

SoftwareSerial mySerial(3, 2);

Adafruit_GPS GPS(&mySerial);


        

#define NUMBER_WAYPOINTS 4   
#define WAYPOINT_DIST_TOLERANE  3   // tolerance in meters to waypoint; once within this tolerance, will advance to the next waypoint
#define GPSECHO  false

// 
////football field
float wplat[] = {41.37217712,41.37261581,41.37252426,41.37258148};
float wplong[]= {-72.09899139,-72.09873962,-72.09896087,-72.09924316};

////around mac
//float wplat[] = {41.371691,41.371654, 41.372069 ,41.372266 };
//float wplong[]= {-72.100191,-72.100556, -72.100487,-72.100412};

//macyeaton parking
//float wplat[] = {41.371671,41.371657};
//float wplong[]= { -72.100240,-72.100510};


//
int waypointNumber = -1;  
int i =0;
int satilites=0;
int originalDistanceToTarget;    // distance to original waypoing when we started navigating to it
int Address = 2;  //This slave is address number 2
int flagdist=0;

float distanceToTarget;           // current distance to target (current waypoint)
float currentLat;
float currentLong;
float targetLat=41.37217712 ;
float targetLong=-72.09899139;
float myLat;
float myLong;
float targetHeading;
float fix=0;
float Gpsheading;
float SOG;

volatile byte* INPUT1FloatPtr;
volatile byte* INPUT2FloatPtr;




boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
      // enter the numebr of way points here (will run from 0 to (n-1))

void setup()
{ Wire.begin(Address);
  Wire.onRequest(requestEvent); // register event
  Serial.begin(9600);
  Serial.println("BOBO GPS");
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);
  delay(1000);
  mySerial.println(PMTK_Q_RELEASE);
  nextWaypoint();
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
        char c = GPS.read();
         // if you want to debug, this is a good time to do it!
         #ifdef UDR0
        if (GPSECHO)
        if (c) UDR0 = c;  
        // writing direct to UDR0 is much much faster than Serial.print 
        // but only one character can be written at a time. 
        #endif
        }

void useInterrupt(boolean v) {
      if (v) {
      // Timer0 is already used for millis() - we'll just interrupt somewhere
      // in the middle and call the "Compare A" function above
      OCR0A = 0xAF;
      TIMSK0 |= _BV(OCIE0A);
      usingInterrupt = true;
      } else {
      // do not call the interrupt function COMPA anymore
      TIMSK0 &= ~_BV(OCIE0A);
      usingInterrupt = false;
              }
        }
  


uint32_t timer = millis();



void loop()                     // run over and over again
{
   GPS_run();
   processGPS();
}

void processGPS(void)
{
    
 myLat = convertDegMinToDecDeg(GPS.latitude);
  myLong = convertDegMinToDecDeg(GPS.longitude);
  Serial.println(myLat,8);
  Serial.println(myLong,8);
  
  if (GPS.latitudeDegrees == 'S')            // make them signed
    myLat = -1 * myLat;
  if (GPS.longitudeDegrees = 'W')  
    myLong = -1 * myLong; 
    
   distanceToTarget = distanceToWaypoint(targetLat,targetLong,myLat,myLong);
   targetHeading=courseToWaypoint(myLat,myLong,targetLat,targetLong);
 
   
}   // processGPS(void)

// converts lat/long from Adafruit degree-minute format to decimal-degrees; requires <math.h> library
float convertDegMinToDecDeg (float degMin) 
{
  double min = 0.0;
  double decDeg = 0.0;
 
  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);
 
  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );
 
  return decDeg;
}
float courseToWaypoint(float lat1, float long1, float lat2, float long2) 
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  float dlon = radians(long2-long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float a1 = sin(dlon) * cos(lat2);
  float a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  return degrees(a2);
}  // courseToWaypoint()



void nextWaypoint(void)
{
  waypointNumber++;
  targetLat =wplat[waypointNumber];
  targetLong =wplong[waypointNumber];
  
  if ( waypointNumber >= NUMBER_WAYPOINTS)    // last waypoint reached? 
    {
    
     Serial.println(F("* LAST WAYPOINT *"));
     
    }
    
   processGPS();
   //distanceToTarget = originalDistanceToTarget = distanceToWaypoint();
   targetHeading=courseToWaypoint(myLat,myLong,targetLat,targetLong);
   
}  // nextWaypoint()


float distanceToWaypoint(float Lat, float Long, float Lat1, float Long1)
{
 float dist;
 float dLat = (float)(Lat1 - Lat);                                    // difference of latitude in 1/10 000 000 degrees
 float dLon = (float)(Long1 - Long) * cos(Lat) ; //
 dist = sqrt(sq(dLat) + sq(dLon)) * 110575;
 float olddist;
  if (dist < WAYPOINT_DIST_TOLERANE)
  {
    if(flagdist==0)
    {
    flagdist=1;
    waypointNumber++;
    }
    if(waypointNumber==NUMBER_WAYPOINTS)
    {
     waypointNumber==100;
    }
    
    olddist = distanceToWaypoint(wplat[waypointNumber-1],wplong[waypointNumber-1],myLat,myLong);
       if ((olddist > (WAYPOINT_DIST_TOLERANE*2)))
       {
       flagdist=0;
       }
 
  }
  return dist;
}


void GPS_run(){
  
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
     if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
      }
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();
  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 1500) { 
    timer = millis(); // reset the timer
    
   
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    if (GPS.fix) {
      fix=1;
      SOG =GPS.speed;
      Gpsheading =GPS.angle;
      satilites=GPS.satellites;
      Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 6);
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees,6);
       Serial.print("Distance: ");Serial.println(distanceToTarget);
      Serial.print("targetHeading: ");Serial.println(targetHeading);
      Serial.print("currentLAT: ");   Serial.println(myLat,4);
     Serial.print("currentLONG: "); Serial.println(myLong,4);
     Serial.print("Target Lat: "); Serial.println(targetLat,4);
     Serial.print("Target Long: ");  Serial.println(targetLong,4);
     }
  }
  
  
  
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
     float myLat;
    float myLong;
       float targetLat;
    float targetLong;
    
  } data = {fix,satilites,waypointNumber,Gpsheading,targetHeading, distanceToTarget,myLat,
             myLong, targetLat, targetLong,};
  Wire.write((unsigned char*)&data, sizeof(data));
}
