#ifndef _CONTROLOUTPUT_H_
#define _CONTROLOUTPUT_H_
#include "Navigation.h"
// Header file for controlling actuator outputs


#define ENABLE_MOTORS
#define LeftMotor 9
#define RightMotor 10

#define velocidad 200
#define P_COEF -10

int update_motors(float heading_error);
int control_output(float current_heading, float target_heading);
int execute_control(float heading_error);
void generate_control(float last_euler, float current_euler, float *current_heading, float Gps_heading, float *heading_error, float target_heading);

#endif // STABILIZATION_H_INCLUDED
