
//Flying Bear
//Small UAV Sbalization Code
//Julian Blanco
//November 2015

#include <Wire.h>  // Included for I2C communication with MPU6050
#include <Servo.h>  // Included to drive output servos/gimbals
#include <PID_v1.h>  // Header file for PID
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP085.h>
#include <SPI.h>
#include <SD.h>


#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.



#define ISR_FREQ 250  // Interrupt service routine frequency (62 to 7800 Hz)

#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();

Adafruit_BMP085 bmp;
 

// Loop variables
uint16_t dt;  // Stores time in micros for gyro integration
uint32_t start;
uint32_t finish;

volatile uint8_t ISR_count = 0;  // Count variable
volatile uint8_t orient_update = 0;  // Update orientation flag
volatile uint8_t servo_update = 0;  // Update servo positions
volatile uint8_t user_update = 0;  // Update airspeed measurement
volatile uint8_t buffer_update = 0;
volatile uint8_t sd_update = 0;
volatile uint8_t barometer_update = 0;
int error;                    // Error code returned when MPU6050 is read
int temp;
long tic;
long toc;
float timed = 0;
Servo rollServo;
Servo pitchServo;
Servo yawServo;

float yawoffset=0;
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

const int chipSelect = 10;

//PID

//Define Variables we'll be connecting to
float rollSetpoint, rollInput, rollServoOutput;
//Define Variables we'll be connecting to
float pitchSetpoint, pitchInput, pitchServoOutput;
//Define Variables we'll be connecting to
float yawSetpoint, yawInput, yawServoOutput;

//Specify the links and initial tuning parameters
float rollKp = 13, rollKi = .1, rollKd = 800000;

//Specify the links and initial tuning parameters
float pitchKp = 13, pitchKi = .1, pitchKd = 800000;

//Specify the links and initial tuning parameters
float yawKp = 6, yawKi = .1, yawKd = 800000;

float rollIntergral = 0;

float pitchIntergral = 0;

float yawIntegral = 0;
//void UpdateOrientation();
//void UpdateServos();
//void UpdateAirspeed();
//void CenterAll();
float derivativeSetpoint = 0;
float intialPressure=0;
float barometerAlt=0;
float pressure=0;
void setup()
{
  Serial.begin(9600);
  rollServo.attach(3);
  pitchServo.attach(6);
  yawServo.attach(9);

  // MPU6050 setup
  Wire.begin();  // Initialize I2C bus

  ISR_init((float)ISR_FREQ);  // Initialize timer 2 at specified interrupt frequency

  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  while (1) {}
  }

  delay(1000);

  int8_t temp = bno.getTemp();

 Serial.println("objeeAirlines UAS -model 1");
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails

  Serial.println("card initialized.");

  Serial.print("Current Temperature: "); Serial.print(temp); Serial.println(" C"); Serial.println("");

  bno.setExtCrystalUse(true);

  finish = micros();

  rollSetpoint = 0;
  pitchSetpoint = -2;
   imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  yawInput=  euler.x();
  Serial.print("Uncal: ");Serial.println(yawInput);

    uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  while(system != 3)
  {
    delay(500);
     bno.getCalibration(&system, &gyro, &accel, &mag);
  }

  yawSetpoint = 1;//set at one deree from intialized

  inputString.reserve(200);
  intialPressure=bmp.readPressure();

    ServoInit();  // Initialize servos
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Serial.print("cal: ");Serial.println(euler.x());
  yawoffset= 0-euler.x(); //yax

   Serial.print("True: ");Serial.println(yawoffset);
}

void loop()
{
  if (orient_update)  UpdateOrientation();
  if (servo_update)  UpdateServos();
   if (user_update)  UpdateUser();
  if (buffer_update) yawSetpoint= (float)newheading((int)yawSetpoint);
  if (barometer_update) UpdateBarometer();
 // if (sd_update)UpdateSD();
  

}

void UpdateSD(){
 
File sdCard = SD.open("datalog.txt", FILE_WRITE);

if (sdCard) {

    sdCard.print(yawInput); sdCard.print(",");
    sdCard.print(rollInput); sdCard.print(",");
    sdCard.print(pitchInput); sdCard.print(",");
    sdCard.print(yawServoOutput); sdCard.print(",");
    sdCard.print(rollServoOutput); sdCard.print(",");
    sdCard.print(pitchServoOutput); sdCard.print(",");
    sdCard.print(pressure); sdCard.print(",");
    sdCard.print(intialPressure); sdCard.print(",");
    sdCard.print(barometerAlt); sdCard.print(",");
    sdCard.close();
  sd_update=0;
        }
else {
    Serial.println("error opening datalog.txt");
  }
}

void UpdateUser() {

    static int x=0;
    x++;
    if (x==20)
    {
    Serial.println("");
    Serial.print("yaw: "); Serial.println(yawInput);
    Serial.print("roll: "); Serial.println(rollInput);
    Serial.print("pitch: "); Serial.println(pitchInput);
     Serial.print("yawsert: "); Serial.println(yawSetpoint);
    Serial.print("rolsetl: "); Serial.println(rollSetpoint);
    Serial.print("pitchset: "); Serial.println(pitchSetpoint);
    Serial.print("Rudder Out: "); Serial.println(yawServoOutput);
    Serial.print("Aileron Out: "); Serial.println(rollServoOutput);
    Serial.print("Elevator Out: "); Serial.println(pitchServoOutput);
    Serial.print("pressure: "); Serial.println(pressure);
    Serial.print("intialPressure: "); Serial.println(intialPressure);
    Serial.print("barometerAlt: "); Serial.println(barometerAlt);
    x=0;
    }
  user_update=0;

}

void UpdateOrientation() {
  
  finish = micros();  // Mark loop time for gyro integration
  timed = finish - start;
  start = finish;
  // Grab Gyro data
  ////////////////////////////////////////////////////////////////////////////////////////////////
  //grabs the gyro data from the sensor
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  yawInput = euler.x()+yawoffset; //yax
  
  pitchInput = euler.y(); //roll
  rollInput = euler.z(); //pitch
  //////////////////////////////////////////////////////////////////////////////////////////////////
  ControlResponse(yawSetpoint, rollSetpoint, pitchSetpoint, yawInput, rollInput, pitchInput);
  orient_update=0;
}

void UpdateServos() {
 
  rollServo.writeMicroseconds(rollServoOutput);
  pitchServo.writeMicroseconds(pitchServoOutput);
  yawServo.writeMicroseconds(yawServoOutput);
  servo_update=0;
}


void UpdateBarometer() {
 
// you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.

  pressure=bmp.readPressure();
  barometerAlt=bmp.readAltitude(intialPressure);
  barometer_update=0;
}

void CenterAll() {
  // Center all servos
  rollServo.writeMicroseconds(1500);
  pitchServo.writeMicroseconds(1500);
  yawServo.writeMicroseconds(1500);
}

void ServoInit() {
  // Initialize control values
  rollServo.writeMicroseconds(1500);
  pitchServo.writeMicroseconds(1500);
  yawServo.writeMicroseconds(1500);
  delay(500);
  rollServo.writeMicroseconds(1900);
  delay(500);
  pitchServo.writeMicroseconds(1900);
  rollServo.writeMicroseconds(1500);
  delay(500);
  pitchServo.writeMicroseconds(1500);
  yawServo.writeMicroseconds(1900);
  rollServo.writeMicroseconds(1100);
  delay(500);
  yawServo.writeMicroseconds(1500);
  pitchServo.writeMicroseconds(1100);
  delay(500);
  yawServo.writeMicroseconds(1100);
  pitchServo.writeMicroseconds(1500);
  delay(1000);
  yawServo.writeMicroseconds(1500);
}


void ControlResponse(float yawTargetValue, float rollTargetValue, float pitchTargetValue, float yawInput, float rollInput, float pitchInput) {
  // Calculate angular error
  float yawError =  control_output(yawInput, yawTargetValue);
  rollTargetValue=yawError*.1;
  if (rollTargetValue <-10) {rollTargetValue =-10; }
  if(rollTargetValue > 10){rollTargetValue=10;} 
  float rollError = rollTargetValue - rollInput;
  float pitchError = pitchTargetValue - pitchInput;
  

  rollServoOutput = rollPID(rollError);
  yawServoOutput = yawPID(yawError);
  pitchServoOutput = pitchPID(pitchError);

}
// Calculate the current error between the current and desired headings
float control_output(float current_heading, float target_heading)
{
// Calculate the heading error given the target and current headings
float heading_error = target_heading - current_heading;

if ( heading_error >= 180)
    heading_error = heading_error - 360;

if ( heading_error < -180)
    heading_error = heading_error + 360;

return (heading_error); 
}

float rollPID(float error) {
  float pResponse = error * rollKp;

  float rollIntegral = 0;
  rollIntegral = error * rollKi;
  static float rollIntegralresponse = 0;
  rollIntegralresponse += error;
  if (rollIntegral > 100)
  {
    rollIntegral = 100;
  }
  if (rollIntegral < -100)
  {
    rollIntegral = -100;
  }

  static float rollError;
  static float rollError1;
  static float rollError2;
  static float rollError3;
  static float rollError4;

  //float derivativeterm = (((rollError4 - rollError3) / timed) + ((rollError3 - rollError2) / timed) + ((rollError2 - rollError1) / timed) + ((rollError1 - rollError) / timed)) * (1 / 4);
   float derivativeterm =(rollError1 - error) / timed;
 
  rollError1 = error;
  rollError2 = rollError1;
  rollError3 = rollError2;
  rollError4 = rollError3;
  
  float derivativeerror = derivativeSetpoint - derivativeterm;
  float derivativeResponse = derivativeerror * rollKd;
 //float response = pResponse + rollIntegralresponse + 1500;
   float response = pResponse + 1500 -derivativeResponse;

  // Bound PWM output
  if ( response > 1800)
    response = 1800;

  else if (response < 1200)
    response = 1200;

  return response;
}
float pitchPID(float error) {

  float pResponse = error * pitchKp;

  float pitchIntegral = 0;
  pitchIntegral = error * pitchKi;
  static float pitchIntegralresponse = 0;
  pitchIntegralresponse += error;
  if (pitchIntegral > 5000)
  {
    pitchIntegral = 5000;
  }
  if (pitchIntegral < -5000)
  {
    pitchIntegral = -5000;
  }

  static float pitchError1;

 // float derivativeterm = (((pitchError4 - pitchError3) / timed) + ((pitchError3 - pitchError2) / timed) + ((pitchError2 - pitchError1) / timed) + ((pitchError1 - error) / timed)) * (1 / 4);
 float derivativeterm =(pitchError1 - error) / timed;
 
  pitchError1 = error;

  
  float derivativeerror = derivativeSetpoint - derivativeterm;
  float derivativeResponse = derivativeerror * pitchKd;
 // float response = response + pitchIntegralresponse + 1500;
  float response = 1500-pResponse +derivativeResponse;

  // Bound PWM output
  if ( response > 1800)
    response = 1800;

  else if (response < 1100)
    response = 1100;

  return response;
}

float yawPID(float error) {
  float pResponse = error * yawKp;
  float yawIntegral = 0;
  yawIntegral = error * yawKi;
  static float yawIntegralresponse = 0;
  yawIntegralresponse += yawIntegral;
  if (yawIntegral > 5000)
  {
    yawIntegral = 5000;
  }
  if (yawIntegral < -5000)
  {
    yawIntegral = -5000;
  }

  static float yawError1;

  //float derivativeterm = (((yawError4 - yawError3) / timed) + ((yawError3 - yawError2) / timed) + ((yawError2 - yawError1) / timed) + ((yawError1 - yawError) / timed)) * (1 / 4);
  float derivativeterm =(yawError1 - error) / timed;
 
  yawError1 = error;
 // yawError2 = yawError1;
  //yawError3 = yawError2;
  //yawError4 = yawError3;
  
  float derivativeerror = derivativeSetpoint - derivativeterm;
  float derivativeResponse = derivativeerror * yawKd;
  //float response = response + yawIntegralresponse + 1500;
    float response = 1500-pResponse +derivativeResponse  ;

  // Bound PWM output
  if ( response > 1800)
    response = 1800;

  else if (response < 1100)
    response = 1100;

  return response;
}

int newheading(int current_heading)
{
   int newhead =0;
if (stringComplete)
{
  Serial.println(inputString);
 newhead=inputString.toInt();
 inputString="";
 Serial.print("newhead: ");Serial.println(newhead);
 stringComplete=0;
}
else
{
  newhead =current_heading;
}
return newhead;
}

ISR(TIMER2_COMPA_vect)  // Interrupt service routine @ 200Hz
{
  if (Serial.available()) 
  {
    char inChar = (char)Serial.read();
    inputString += inChar;
      if (inChar == '\n') 
      {
      stringComplete = true;
      }
  }

  ISR_count++;
  if ((ISR_count % 3)==0) orient_update = 1;  // Update orientation data
  if ((ISR_count % 200)==0) buffer_update = 1; 
  if ((ISR_count % 8) == 0)  servo_update = 1;  // Update servo positions
  if ((ISR_count % 400) == 0)  user_update =1;  // Sample Airspeed
  if ((ISR_count % 200)==0) barometer_update = 1;
  if ((ISR_count % 400)==0) sd_update = 1;
  if (ISR_count >= 500)  ISR_count = 0;  // Reset ISR_count

}


void ISR_init(float frequency)
{
  /*  pg. 162
  http://www.atmel.com/images/doc8161.pdf
   */

  cli();                    // Disable global interrupts

  TCCR2A = 0;               // Clear Timer A register
  TCCR2B = 0;               // Clear Timer B register
  TCNT2 = 0;                // Set counter value to 0
  OCR2A = (uint8_t)((16000000 / (frequency * 1024)) - 1);  // Reset value
  TCCR2A |= (1 << WGM21);   // Turn on CTC mode
  TCCR2B = TCCR2B | (1 << CS22) | (1 << CS21) | (1 << CS20);  // 1024 Prescaler
  TIMSK2 |= (1 << OCIE2A);  // Enable compare match interrupt
  sei();                    // Enable global interrupts
}

