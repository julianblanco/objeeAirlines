
//Flying Bear
//Small UAV Sbalization Code
//Julian Blanco
//November 2015

#include <Wire.h>  // Included for I2C communication with MPU6050
#include <Servo.h>  // Included to drive output servos/gimbals
#include <PID_v1.h>  // Header file for PID
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define ISR_FREQ 250  // Interrupt service routine frequency (62 to 7800 Hz)

#define BNO055_SAMPLERATE_DELAY_MS (100)


#define rollMaxAllowed 30

Adafruit_BNO055 bno = Adafruit_BNO055();


// Loop variables
uint16_t dt;  // Stores time in micros for gyro integration
uint32_t start;
uint32_t finish;

volatile uint8_t ISR_count = 0;  // Count variable
volatile uint8_t orient_update = 0;  // Update orientation flag
volatile uint8_t servo_update = 0;  // Update servo positions
volatile uint8_t user_update = 0;  // Update airspeed measurement

int error;                    // Error code returned when MPU6050 is read
int temp;
long tic;
long toc;
float timed = 0;
Servo rollServo;
Servo pitchServo;
Servo yawServo;


//PID

//Define Variables we'll be connecting to
double rollSetpoint, rollInput, rollServoOutput;
//Define Variables we'll be connecting to
double pitchSetpoint, pitchInput, pitchServoOutput;
//Define Variables we'll be connecting to
double yawSetpoint, yawInput, yawServoOutput;

//Specify the links and initial tuning parameters
double rollKp = 13, rollKi = .1, rollKd = .5;

//Specify the links and initial tuning parameters
double pitchKp = 13, pitchKi = .1, pitchKd = 1;

//Specify the links and initial tuning parameters
double yawKp = 10, yawKi = .1, yawKd = .5;

float rollIntergral = 0;

float pitchIntergral = 0;

float yawIntegral = 0;

float pitchResponse =0;

float yawResponse =0;
//void UpdateOrientation();
//void UpdateServos();
//void UpdateAirspeed();
//void CenterAll();
float derivativeSetpoint = 0;

void setup()
{
  Serial.begin(9600);
  rollServo.attach(3);
  pitchServo.attach(6);
  yawServo.attach(9);

  // MPU6050 setup
  Wire.begin();  // Initialize I2C bus
  ServoInit();  // Initialize servos
  ISR_init((float)ISR_FREQ);  // Initialize timer 2 at specified interrupt frequency

  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: "); Serial.print(temp); Serial.println(" C"); Serial.println("");

  bno.setExtCrystalUse(true);

  finish = micros();

  rollSetpoint = 0;
  pitchSetpoint = -2;
  yawSetpoint = 0;

}

void loop()
{
  if (orient_update)  UpdateOrientation();
  if (servo_update)  UpdateServos();
  if (user_update)  UpdateUser();
  // Other loop stuff

}

void UpdateUser() {

    Serial.println("");
    Serial.print("yaw: "); Serial.println(yawInput);
    Serial.print("roll: "); Serial.println(rollInput);
    Serial.print("pitch: "); Serial.println(pitchInput);
    Serial.print("Rudder Out: "); Serial.println(yawServoOutput);
    Serial.print("Aileron Out: "); Serial.println(rollServoOutput);
    Serial.print("Elevator Out: "); Serial.println(pitchServoOutput);
 
  user_update=0;

}

void UpdateOrientation() {
  orient_update = 0;

  finish = micros();  // Mark loop time for gyro integration
  dt = finish - start;
  start = finish;
  // Grab Gyro data
  ////////////////////////////////////////////////////////////////////////////////////////////////
  //grabs the gyro data from the sensor
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  yawInput = euler.x(); //yax
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



void CenterAll() {
  // Center all servos
  rollServo.writeMicroseconds(1500);
  pitchServo.writeMicroseconds(1500);
  yawServo.writeMicroseconds(1500);
}



void ControlResponse(float yawTargetValue, float rollTargetValue, float pitchTargetValue, float yawInput, float rollInput, float pitchInput) {
  // Calculate angular error
  float yawError =  control_output(yawInput, yawTargetValue);
  //Next Three lines bank the plane based on yaw error, and set satuaration point.
  rollTargetValue =1.3*yawError; 
  if (rollTargetValue > rollMaxAllowed) rollTargetValue=rollMaxAllowed;
  if (rollTargetValue < -1*rollMaxAllowed) rollTargetValue=-1*rollMaxAllowed;
    
  float rollError = rollTargetValue - rollInput;
  float pitchError = pitchTargetValue - pitchInput;
  

 
  yawResponse = yawPID(yawError);
  pitchResponse = pitchPID(pitchError);

  rollServoOutput = rollPID(rollError);
  yawServoOutput=cos(rollInput)*yawResponse + sin(rollInput)*pitchResponse;
  pitchServoOutput=sin(rollInput)*yawResponse + cos(rollInput)*pitchResponse;

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

  float derivativeterm = (((rollError4 - rollError3) / timed) + ((rollError3 - rollError2) / timed) + ((rollError2 - rollError1) / timed) + ((rollError1 - rollError) / timed)) * (1 / 4);
  rollError4 = rollError3;
  rollError3 = rollError2;
  rollError2 = rollError1;
  rollError1 = rollError;
  float derivativeerror = derivativeSetpoint - derivativeterm;
  float derivativeResponse = derivativeerror * rollKd;
 //float response = pResponse + rollIntegralresponse + 1500;
   float response = pResponse + 1500;

  // Bound PWM output
  if ( response > 1700)
    response = 1700;

  else if (response < 1300)
    response = 1300;

  return response;
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

  static float pitchError;
  static float pitchError1;
  static float pitchError2;
  static float pitchError3;
  static float pitchError4;

  float derivativeterm = (((pitchError4 - pitchError3) / timed) + ((pitchError3 - pitchError2) / timed) + ((pitchError2 - pitchError1) / timed) + ((pitchError1 - pitchError) / timed)) * (1 / 4);
  pitchError4 = pitchError3;
  pitchError3 = pitchError2;
  pitchError2 = pitchError1;
  pitchError1 = pitchError;
  float derivativeerror = derivativeSetpoint - derivativeterm;
  float derivativeResponse = derivativeerror * pitchKd;
 // float response = response + pitchIntegralresponse + 1500;
  float response = 1500-pResponse ;

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

  static float yawError;
  static float yawError1;
  static float yawError2;
  static float yawError3;
  static float yawError4;

  float derivativeterm = (((yawError4 - yawError3) / timed) + ((yawError3 - yawError2) / timed) + ((yawError2 - yawError1) / timed) + ((yawError1 - yawError) / timed)) * (1 / 4);
  yawError4 = yawError3;
  yawError3 = yawError2;
  yawError2 = yawError1;
  yawError1 = yawError;
  float derivativeerror = derivativeSetpoint - derivativeterm;
  float derivativeResponse = derivativeerror * yawKd;
  //float response = response + yawIntegralresponse + 1500;
    float response = 1500-pResponse  ;

  // Bound PWM output
  if ( response > 1800)
    response = 1800;

  else if (response < 1100)
    response = 1100;

  return response;
}


ISR(TIMER2_COMPA_vect)  // Interrupt service routine @ 200Hz
{
  ISR_count++;
  if ((ISR_count % 3)==0) orient_update = 1;  // Update orientation data
  if ((ISR_count % 8) == 0)  servo_update = 1;  // Update servo positions
  if ((ISR_count % 400) == 0)  user_update = 1;  // Sample Airspeed
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

