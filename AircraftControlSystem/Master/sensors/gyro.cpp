#include "gyro.h"


void sampleGyro(Adafruit_BNO055 &gyroIMU){
    /* Display the floating  point data */
    imu::Vector<3> euler = gyroIMU.getVector(Adafruit_BNO055::VECTOR_EULER);
    float headingInput=euler.x()-headingOffset;

    if (headingInput >= 360)
    {
          headingInput -= 360;
        }
    else 
    {if (headingInput < 0)headingInput += 360;
    }
    trueHeading=headingInput;
    pitchInput=euler.z();
    //rollInput=fmod((euler.z()+(360+90)), 360)-180;
    rollInput=euler.y();
}
