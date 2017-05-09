
// Evan Twarog....

#include <Servo.h>
#include "Adafruit_BNO055.h"
float trueHeading = 0;
float desiredHeading =0;
float proportionalConstant1 = 25;
float proportionalConstant2 = 10;
float proportionalConstant3 = 5;
float response = 0;
float error = 0;
float yawInput = 0;
float yawOffset = 0;

Servo myServo;
Servo Throttle;


Adafruit_BNO055 bno = Adafruit_BNO055();


void setup(void) 
{
  Serial.begin(9600);
  Wire.begin();  // Initialize I2C bus
  if(!bno.begin())while(1){};
  bno.setExtCrystalUse(true);
  myServo.attach(9);
  Throttle.attach(10);
  Throttle.writeMicroseconds(1500);
  delay(2000);
  Throttle.writeMicroseconds(1550);

}

void loop()
{

trueHeading = SampleGyroX();
error = angular_diff(desiredHeading,trueHeading);

if (abs(error)<5) response = 1500-error*proportionalConstant1;
if ((abs(error)>5) && (abs(error)<20) ) response = 1500-error*proportionalConstant2;
if (abs(error)>20) response = 1500-error*proportionalConstant3;

float outputSteering  = constrain(response,1000,2000);
myServo.writeMicroseconds(outputSteering);

Serial.print(trueHeading);Serial.print(",");
Serial.println(error);

delay(100);
}




float SampleGyroX(){
    /* Display the floating  point data */
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    yawInput=euler.x()-yawOffset;

    if (yawInput >= 360)
    {
          yawInput -= 360;
        }
    else 
    {if (yawInput < 0)yawInput += 360;
    }

    return yawInput;
}


// Find the shortest signed angular difference between a target and source angle
float angular_diff(float target_angle, float source_angle)
{
        // Find simple difference
        float diff = target_angle - source_angle;
        if (diff > 180)
                diff -= 360;
        else if (diff < -180)
                diff += 360;

        return(diff);
}

