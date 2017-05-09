#ifndef _CONTROLOUTPUT_H_
#define _CONTROLOUTPUT_H_




#define velocidad 200


float update_steering(float heading_error,float speed);
float update_motors(float speed_error);
float speedcalc(float heading_error,float distance2wp,float maxspeed);
float control_output(float current_heading, float target_heading);
#endif // STABILIZATION_H_INCLUDED
