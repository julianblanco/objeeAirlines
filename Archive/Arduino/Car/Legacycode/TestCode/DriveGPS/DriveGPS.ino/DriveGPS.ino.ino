#include <Wire.h>                                 // used by: motor driver
#include <Adafruit_Sensor.h>                      // part of mag sensor
#include <math.h>                                 // used by: GPS
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>    
#define ENABLE_MOTORS
#define LeftMotor 9
#define RightMotor 10

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

//#define ENABLE_MOTORS
int x = 0;
int velocidad = 190;
int useroverrideFlag =0;
int counter =0;
 

float tiempo =0;
float P_coef = 3;
float GpsHeading;
float currentHeading = 0;
float targetHeading = 0;
float GyroHeading =0;
float lastEuler=0;
float currentEuler = 0;
float headingError=0;
float SpeedOverGround =0;
float temp_heading =0;
float currentLat;
float currentLong;
float targetLat;
float targetLong;
float D2Tar=0;
float FILTER_COEF =0.3;
uint8_t fix=0;
uint8_t satilites=0;
uint8_t waypointNumber;
float desiredgyro;

void setup() {
  Serial.begin(9600);
  
  Serial.println("Inialize GYRO");
  delay(8000);
  /* Initialise the sensor */
    if(!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }
    
    bno.setExtCrystalUse(true);
    
  
    // I2C start as master
    Wire.begin();
        // Get new Euler reading
        
        
        
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    targetHeading = euler.x();
    tiempo= millis();
    Update_GPS_Heading();
    while(fix==0)
    {
      Serial.print("Waiting for fix: ");
      Update_GPS_Heading();
      delay(3000);
      counter=counter+3;
      Serial.println(counter);
      analogWrite(LeftMotor, 0);
      analogWrite(RightMotor, 0);
    }
    while(tiempo <10000)
    {
    drive_gyro_only();
    tiempo = millis();
    
    }

}
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  
 
  find_drift_error();
  update_motors();
  Update_User();
  user_override();
  Update_GPS_Heading(); 

    delay(200);
}
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
   

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
       targetHeading = inString.toFloat();
       useroverrideFlag =1;
     }
  }
}

void drive_gyro_only()
{
   // Move Euler reading from last loop
    lastEuler = currentEuler;
    
    // Get new Euler reading
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    currentEuler = euler.x();

    //currentHeading =  (lastEuler - currentEuler); // update state estimation
      currentHeading=euler.x();
     currentHeading =correct_wrap(currentHeading);
   
   
     // Calculate heading error to apply fast corrections
    headingError = targetHeading - GyroHeading;
    
    if( headingError >= 180)
        headingError=headingError-360;
    
    if(headingError < -180)
        headingError=headingError+360;
    
     update_motors();
}

void find_drift_error(void)
{
  // Move Euler reading from last loop
    lastEuler = currentEuler;
    
    // Get new Euler reading
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    currentEuler = euler.x();
    GyroHeading=GpsHeading+temp_heading;
    currentHeading=correct_wrap(GyroHeading);
    // Calculate heading error to apply fast corrections
    headingError = targetHeading - currentHeading;
    
    
    if( headingError >= 180)
        headingError=headingError-360;
    
    if(headingError < -180)
        headingError=headingError+360;
    
    x++;
  
}

float i2c_read_float(uint8_t addr)
{
  float fl = 0.f;
  
  Wire.beginTransmission(addr);
  Wire.requestFrom(addr, 4);
  for(uint8_t* p =  (uint8_t*)&fl; p < ((uint8_t*)&fl + 4); ++p)
    *p = Wire.read();
  Wire.endTransmission();
  
  return fl;
}

void Update_GPS_Heading()
{
if(x==50);
{
  Wire.beginTransmission(2);
  Wire.requestFrom(2, 31);
  fix = Wire.read();
  satilites =Wire.read();
  waypointNumber= Wire.read();
  //GPS HEADING
  for(uint8_t* p = (uint8_t*)&GpsHeading; p < ((uint8_t*)&GpsHeading + 4); ++p)
    *p = Wire.read();
  //TARGET HEADING
  for(uint8_t* p = (uint8_t*)&targetHeading; p < ((uint8_t*)&targetHeading + 4); ++p)
    *p = Wire.read();
    //distance
    for(uint8_t* p = (uint8_t*)&D2Tar; p < ((uint8_t*)&D2Tar + 4); ++p)
    *p = Wire.read();
    //CURRENTLAT
  for(uint8_t* p = (uint8_t*)&currentLat; p < ((uint8_t*)&currentLat + 4); ++p)
    *p = Wire.read();
 //   CURRENT LONG
  for(uint8_t* p = (uint8_t*)&currentLong; p < ((uint8_t*)&currentLong + 4); ++p)
    *p = Wire.read();
    //TARGET LAT
  for(uint8_t* p = (uint8_t*)&targetLat; p < ((uint8_t*)&targetLat + 4); ++p)
    *p = Wire.read();
    //TARGETLONG
  for(uint8_t* p = (uint8_t*)&targetLong; p < ((uint8_t*)&targetLong + 4); ++p)
    *p = Wire.read();
    //SOG
 // for(uint8_t* p = (uint8_t*)&SpeedOverGround; p < ((uint8_t*)&SpeedOverGround + 4); ++p)
 //   *p = Wire.read();
 
// currentHeading = FILTER_COEF*GpsHeading + (1-FILTER_COEF)*currentHeading; // "anchor" state estimation to GPS heading
temp_heading = ( GpsHeading-GyroHeading ) ;

  //END
  Wire.endTransmission();



 
 
}
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
    
    if (PWM_right < 150)
        PWM_right = 150;
    if (PWM_left < 150)
        PWM_left = 150;
        
    // Write responses to each motor
    #ifdef ENABLE_MOTORS
      analogWrite(LeftMotor, PWM_left);
      analogWrite(RightMotor, PWM_right);
    #endif
    
    
    if(waypointNumber==100)
    {
      velocidad=0;
    Serial.println("*********************************************************");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("            Mission Complete                       ");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("*********************************************************");
    
    }
  
}

void Gps_aquire_course()
{
    Serial.println("Drive test");
    
    drive_gyro_only();
    
}


    
    void user_override()
    {
       while(useroverrideFlag ==1)
             {
                counter =0;
                drive_gyro_only(); 
                delay(100);
                counter++;
   
                 if(counter>100)
                 {
                 useroverrideFlag ==1;
                  }//end if
   
             }//end while
   
    }
    
    float correct_wrap(float current_heading)
{

  // Correct 360 deg wrap around
  if ( current_heading > 360)
    current_heading = current_heading - 360;

  if (current_heading < 0)
    current_heading = current_heading + 360;

  return (current_heading);
}


