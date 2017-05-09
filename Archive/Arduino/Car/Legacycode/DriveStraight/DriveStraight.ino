#include <Wire.h>                                 // used by: motor driver
#include <Adafruit_Sensor.h>                      // part of mag sensor
#include <math.h>                                 // used by: GPS
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>                         // GPS
#include <SoftwareSerial.h>                       // used by: GPS
#include <math.h>    

#define LeftMotor 9
#define RightMotor 10

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

// GPS Navigation
#define NO_GPS_WAIT
#define GPSECHO false           // set to TRUE for GPS debugging if needed
//#define GPSECHO true           // set to TRUE for GPS debugging if needed
SoftwareSerial mySerial(3, 2);    // digital pins 8(RX) & 7(TX)
Adafruit_GPS GPS(&mySerial);
boolean usingInterrupt = false;
float currentLat,
currentLong,
targetLat,
targetLong;
int distanceToTarget,            // current distance to target (current waypoint)
originalDistanceToTarget;    // distance to original waypoing when we started navigating to it





float GpsHeading;
float P_coef = 3;
float lastEuler;
float currentEuler = 0;
float headingError;
int velocidad = 150;
float currentHeading = 0;
float targetHeading = 0;
float GyroHeading =0;
float SpeedOverGround =0;
void setup() {
  Serial.begin(9600);
  
  Serial.println("Inialize BNO055");
  
  /* Initialise the sensor */
    if(!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }
    
    bno.setExtCrystalUse(true);
    
    Serial.println("Drive test");
    analogWrite(LeftMotor, 150);
    analogWrite(RightMotor, 150);
    delay(1000);
    analogWrite(LeftMotor, 0);
    analogWrite(RightMotor, 0);
    delay(2000);
    
    
    
      GPS.begin(9600);                                // 9600 NMEA default speed
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);     // turns on RMC and GGA (fix data)
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);       // 1 Hz update rate
    GPS.sendCommand(PGCMD_NOANTENNA);                // turn off antenna status info
    useInterrupt(false);                            // use interrupt to constantly pull data from GPS
    delay(1000);
    
    //
    // Wait for GPS to get signal
    #ifndef NO_GPS_WAIT
    Serial.println(F("Waiting for GPS"));
    unsigned long startTime = millis();
    while (!GPS.fix)                      // wait for fix, updating display with each new NMEA sentence received
    {
        Serial.print(F("Wait Time: "));
        Serial.println((int) (millis() - startTime) / 1000);     // show how long we have waited  
        if (GPS.newNMEAreceived())
            GPS.parse(GPS.lastNMEA());
            
        delay(1000);
    } // while (!GPS.fix)
    //delay(1000);
    #endif
    
}









void loop() {
  find_drift_error();
  update_motors();
  UpdateGPS();
  //delay(100);
}

void UpdateGPS(void)
{
    
    GPS.read();
    
    
    // Process GPS 
    if (GPS.newNMEAreceived())               // check for updated GPS information
    {                                      
        if(GPS.parse(GPS.lastNMEA()) )      // if we successfully parse it, update our data fields
            processGPS();   
    } 
    
    
}
void processGPS(void)
{
    currentLat = convertDegMinToDecDeg(GPS.latitude);
    currentLong = convertDegMinToDecDeg(GPS.longitude);
    
    if (GPS.lat == 'S')            // make them signed
        currentLat = -1 * currentLat;
    if (GPS.lon = 'W')  
        currentLong = -1 * currentLong; 
    
    SpeedOverGround = (0.5*SpeedOverGround) + (0.5*GPS.speed);
    Serial.print("Speed: ");
    Serial.println(SpeedOverGround);
}   // processGPS(void)
void serialEvent()
{
  String inString = "";
  
  while(Serial.available() > 0)
  {
    int inChar = Serial.read();
     if (isDigit(inChar)) {
       // convert the incoming byte to a char
       // and add it to the string:
       inString += (char)inChar;
     }
     if (inChar == '\n') {
       targetHeading = inString.toInt();
     }
  }
}
    // converts lat/long from Adafruit degree-minute format to decimal-degrees; requires <math.h> library
double convertDegMinToDecDeg (float degMin) 
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

void useInterrupt(boolean v) 
{
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
void find_drift_error(void)
{
  // Move Euler reading from last loop
    lastEuler = currentEuler;
    
    // Get new Euler reading
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    currentEuler = euler.x();
    
    // Update current heading with Euler difference between loops
    GyroHeading = (currentEuler - lastEuler) + currentHeading;
    GpsHeading =GPS.angle;
    Serial.print("GPS: ");
    Serial.println(GpsHeading);
    currentHeading = (0.2*GyroHeading) + (0.8*GpsHeading);
    
    // Calculate heading error to apply fast corrections
    headingError = targetHeading - currentHeading;
    
    if( headingError >= 180)
        headingError=headingError-360;
    
    if(headingError < -180)
        headingError=headingError+360;
    
    
    Serial.print("Heading error: "); Serial.println(headingError);
}


void update_motors(void)
{
  // Calculate left/right motor response (positive -> right)
    float response = P_coef * headingError;
    int PWM_left = velocidad - (int)response;
    int PWM_right = velocidad + (int)response;
    
    
    // Bound PWM values
    if (PWM_left > 255)
        PWM_left = 255;
    else if (PWM_left < 0)
        PWM_left = 0;
    
    if (PWM_right > 255)
        PWM_right = 255;
    else if (PWM_right < 0)
        PWM_right = 0;
    
    
    // Write responses to each motor
    analogWrite(LeftMotor, PWM_left);
    analogWrite(RightMotor, PWM_right);
    
    Serial.print("Left motor PWM: "); Serial.println(PWM_left);
    Serial.print("Right motor PWM: "); Serial.println(PWM_right);
}
