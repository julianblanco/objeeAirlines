//#include "ControlOutputs.h"
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


#define ENABLE_MOTORS
#define PWM_MOTOR

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();



class driveStraight
{
  public:

    Servo Steer;
    Servo gas;
    float tiempo = millis();
    float current_heading ;
    float target_heading;
    float error = 0;
    int steer;
};


void setup()
{
  Steer.attach(9);  // attaches the servo on pin 9 to the servo object
  gas.attach(10);
  gas.writeMicroseconds(1500);
  Steer.writeMicroseconds(1500);
  delay(1000);
  Serial.begin(9600);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);
  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  target_heading = euler.x();
}


void loop()
{
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  current_heading = euler.x();
  error = control_output(current_heading, target_heading);

  // Calculate and bound control response
   driveStraight.steer = (P_COEF * (int)error) + 1500;
  if (driveStraight.steer < 1200)
  {
    driveStraight.steer = 1200;
  }
  else if (driveStraight.steer > 1800)
  {
    driveStraight.steer = 1800;
  }
  
  Serial.println(error);
  //steer = update_motors(error);
  Serial.println(driveStraight.steer);
  Steer.writeMicroseconds(steer);
  gas.writeMicroseconds(1600);

}

