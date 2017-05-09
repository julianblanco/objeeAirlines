#ifndef _CONTROLOUTPUT_H_
#define _CONTROLOUTPUT_H_
#include <Arduino.h>



#define steeringKp 3
#define steeringKi 1
#define steeringKd 2
#define derivativeSetpoint 0

float update_motors(float heading_error,float timed);
float control_output(float current_heading, float target_heading);
#endif // STABILIZATION_H_INCLUDED
