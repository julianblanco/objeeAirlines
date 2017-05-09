#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>
#include "IIRFilter.h"
#include "Navigation.h"
#include "ControlOutputs.h"
#include  "latgenerator.h"
#define mySerial Serial1
Adafruit_GPS GPS(&mySerial);
#define pwmSerial Serial2
#define Matlab Serial3

#define GPSECHO  false
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();

//football field
//float wplat[] = {41.372158,41.372158,41.372146,41.372394,41.372529,41.372018,41.372664,41.372568};
//float wplong[]= { -72.099016,-72.099016,-72.098811,-72.099251, -72.098714,-72.099025, -72.098955,-72.099287};

//float wplat[] = {41.372315,41.372052,41.372316,41.372064,41.372174,41.372145 ,41.372462,41.372160 ,41.371996,41.372620};
//float wplong[]= {-72.098873,-72.098903,-72.099149,-72.099024, -72.099200, -72.098829, -72.098972,-72.099014, -72.099035, -72.099271};
float wplat[] = {41.375567,41.375567,41.375164,41.375269,41.375527,41.375613,41.375069};
float wplong[] = {-72.097816,-72.097816,-72.097336,-72.097759,-72.098202,-72.097436,-72.098100};


//Navigation Variables
/////////////////////////////////////////////////////////
float trueHeading=0;
float targetHeading=0;
float Gpsheading=0;
float currentEuler;
float eulerLast=0;
float headingError=0;

float GpsSpeed=0;
float GpsAltitude=0;
float GpsFixQual=0;

float distanceToTarget=0;
float lastdistnace=0;
float precision = 1;                                        // current distance to target (current waypoint)

int Fix=0;
bool new_GPS_data = 0;
int GpsSat=0;
int steer;
int waypointNumber = 0;  
int flagdist=0;


float targetLat=41.371671 ;
float targetLong=-72.100240;

float myLat=41.3;
float myLong=-72.100240;


float lasttargetLat=41.371671 ;
float lasttargetLong=-72.100240;
float currenttime=0;
float lasttime=0;
float loopspeed=0;
float course =0;


//football field
//float wplat[] = {41.372158,41.372158,41.372146,41.372394,41.372529,41.372018,41.372664,41.372568};
//float wplong[]= { -72.099016,-72.099016,-72.098811,-72.099251, -72.098714,-72.099025, -72.098955,-72.099287};





/**************************************************************************/
/*
Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void) 
{

  Serial.begin(9600);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");


  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");Serial.print(temp);Serial.println(" C"); Serial.println("");

  bno.setExtCrystalUse(true);
  startTimer(TC1, 0, TC3_IRQn, 2000); 
   Serial.println("Adafruit GPS library basic test!");
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  mySerial.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
  mySerial.println(PMTK_Q_RELEASE);
  Matlab.begin(9600);
  Matlab.println('a');
  pwmSerial.begin(115200);
  waypointing();
  trueHeading= courseToWaypoint(myLat, myLong, targetLat, targetLong);

// Steer straight initially
  pwmSerial.println(90);

}

void loop(void) 


{
    lasttime=currenttime;
    currenttime=millis();
    loopspeed = (1000/(currenttime-lasttime));

    // Grab Gyro data
    ////////////////////////////////////////////////////////////////////////////////////////////////
    //grabs the gyro data from the sensor
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    currentEuler=euler.x();
    //////////////////////////////////////////////////////////////////////////////////////////////////


    //Aquire the GPS VALUES (PARSE)
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    //Uses ADAFRUIT Library to parse the GPS Data
    if (GPS.newNMEAreceived()) {
        if (GPS.parse(GPS.lastNMEA()))                          // this also sets the newNMEAreceived() flag to false
        {
            new_GPS_data = 1;
            myLat=GPS.latitudeDegrees;
            myLong=GPS.longitudeDegrees;
            GpsSpeed=GPS.speed;
            Gpsheading=GPS.angle;
            GpsAltitude=GPS.altitude;
            GpsSat=GPS.satellites;
        }
        else
        {
            new_GPS_data = 0;
        }

        Fix=GPS.fix;
    }
    // Serial.print("Fix: "); Serial.print((int)GPS.fix);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////


    // IIR Filter
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    // Only use GPS heading data if there's a new fix and vehicle is travelling fast enough
    bool use_GPS = 0;
    if(new_GPS_data > 0 && GpsSpeed > 2)
    {
        use_GPS = 1;
        new_GPS_data=new_GPS_data - 1;
    }
    else
    {
        use_GPS = 0;
    }

    if(waypointNumber >1)
    {
         lasttargetLat = wplat[waypointNumber-1];
         lasttargetLong = wplong[waypointNumber-1];
         lastdistnace = distanceToWaypoint(myLat, myLong, lasttargetLat, lasttargetLong);
         if(lastdistnace<4)
         {
             use_GPS = 0;
         }
    }


    // Update heading estimate with gyro and GPS data
    trueHeading=update_heading_estimate(Gpsheading,currentEuler,trueHeading,use_GPS);

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    //Navigates
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // calculates the heading to the Waypoing
    targetHeading = courseToWaypoint(myLat, myLong, targetLat, targetLong);
    //calculates the distance to the waypoint
    distanceToTarget = distanceToWaypoint(myLat, myLong, targetLat, targetLong);

    //checks to see if we are close enough to the waypoint and 
    waypointing();
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




    //Creates the response for the motors
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //heading error uses PID to compute response to correct the heading error
    headingError=  control_output(trueHeading, targetHeading);
    //steering uses heading errror and creates a command to send to the motors based on platform
    steer = update_motors(headingError);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    pwmSerial.println(steer);

    if (Fix)
    {
      notify_user();
    }


    //serial prints data
    
    Matlabsend();
    delay(10);
}                                                           //end loop



// Serial print to com port and xbee to matlab
////////////////////////////////////////////////////////////
void notify_user()
{
    static int x =0;
    x++;
    if(x>200)
    {
        Serial.print(myLat,6);Serial.print(',');
        Serial.print(myLong,6);Serial.print(',');
        Serial.print(targetLat,6);Serial.print(',');
        Serial.print(targetLong,6);Serial.print(',');
        Serial.print(targetHeading,4);Serial.print(',');
        Serial.print(Gpsheading,4);Serial.print(',');
        Serial.print(currentEuler,4);Serial.print(',');
        Serial.print(trueHeading,4);Serial.print(',');
        Serial.print(headingError,4);Serial.print(',');
        Serial.print(GpsSpeed);Serial.print(',');
        Serial.print(waypointNumber);Serial.print(',');
        Serial.print(distanceToTarget,4);Serial.print(',');
        Serial.print(steer);Serial.print(',');
        Serial.print(loopspeed);Serial.print(',');
        Serial.print(GPS.hour);Serial.print(',');
        Serial.print(GPS.minute);Serial.print(',');
        Serial.println(GPS.seconds);
        x=0;
    }
}

void Matlabsend()
{
    static int y =0;
    y++;
    if(y>200)
    {
        Matlab.println('a');
        Matlab.print("!");  Matlab.println(targetHeading,4);
        Matlab.print("@"); Matlab.println(trueHeading,4);
        Matlab.print("#");Matlab.println(Gpsheading,4);
        Matlab.print("$");Matlab.println(currentEuler,4);
        Matlab.print("%"); Matlab.println(headingError,4);
        Matlab.print("^");    Matlab.println(Fix);
        Matlab.print("&");   Matlab.println(myLat,4);
        Matlab.print("*"); Matlab.println(myLong,4);
        Matlab.print("("); Matlab.println(targetLat,4);
        Matlab.print(")");  Matlab.println(targetLong,4);
        Matlab.print("q"); Matlab.println(GpsSpeed);
        Matlab.print("w"); Matlab.println((waypointNumber));
        Matlab.print("e"); Matlab.println(distanceToTarget,4);
        Matlab.print("r");Matlab.println(GPS.hour);
        Matlab.print("t");Matlab.println(GPS.minute);
        Matlab.print("y");Matlab.println(GPS.seconds);
        // // Matlab.print("Satilites: "); Matlab.println(satilites); 
        // Matlab.print(" quality: "); Matlab.println((int)GPS.fixquality);  
        y=0;
    }
}

void waypointing()
{
    static int wpcounter =0;
    if(distanceToTarget<2)
    {
        wpcounter++;
        
        if (wpcounter>2 &&wpcounter<14)
        {
             wplat[0] = targetLat;
             wplong[0] = targetLong;
             wplat[1]=latgenerator(wplat[0],wplong[0],15,course  );
             wplong[1]=longgenerator(wplat[0],wplong[0],15,course );
             waypointNumber=1;
             course = createcourse(course,1);
             //Serial.println(wpcounter);
        }
       else if(wpcounter>13&&wpcounter<17)
        {
             wplat[0] = targetLat;
             wplong[0] = targetLong;
             wplat[1]=latgenerator(wplat[0],wplong[0],10,0 );
             wplong[1]=longgenerator(wplat[0],wplong[0],10,0);
             waypointNumber=1;
           //  Serial.println(wpcounter);
        } 
        else
        {
        waypointNumber=waypointNumber+1;
        }

    targetLat = wplat[waypointNumber];
    targetLong =wplong[waypointNumber];
   
    }
    
    
}


//Arduin Due Interupt Service Routine Code
/////////////////////////////////////////////////////////////////////
void TC3_Handler()
{

    TC_GetStatus(TC1, 0);

    char c = GPS.read();      
}

void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency)
{
    pmc_set_writeprotect(false);
    pmc_enable_periph_clk((uint32_t)irq);
    TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
    uint32_t rc = VARIANT_MCK/128/frequency;            //128 because we selected TIMER_CLOCK4 above
    TC_SetRA(tc, channel, rc/2);                        //50% high, 50% low
    TC_SetRC(tc, channel, rc);
    TC_Start(tc, channel);
    tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
    tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
    NVIC_EnableIRQ(irq);
}
