#include "gyro.h"


void SampleGyro(Adafruit_BNO055 &gyroIMU){
    /* Display the floating  point data */
    imu::Vector<3> euler = gyroIMU.getVector(Adafruit_BNO055::VECTOR_EULER);
    yawInput=euler.x()-yawOffset;

    if (yawInput >= 360)
    {
          yawInput -= 360;
        }
    else 
    {if (yawInput < 0)yawInput += 360;
    }
    pitchInput=euler.y();
    //rollInput=fmod((euler.z()+(360+90)), 360)-180;
    rollInput=euler.z();
    uint8_t system, gyro, accel, mag = 0;
    gyroIMU.getCalibration(&system, &gyro, &accel, &mag);
    calibration=system;
}

float SampleGyroX(Adafruit_BNO055 &gyroIMU){
    /* Display the floating  point data */
    imu::Vector<3> euler = gyroIMU.getVector(Adafruit_BNO055::VECTOR_EULER);
    yawInput=euler.x()-yawOffset;

    if (yawInput >= 360)
    {
          yawInput -= 360;
        }
    else 
    {if (yawInput < 0)yawInput += 360;
    }
}

