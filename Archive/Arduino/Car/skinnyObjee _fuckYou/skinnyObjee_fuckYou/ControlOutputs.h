#ifndef _CONTROLOUTPUT_H_
#define _CONTROLOUTPUT_H_




#define velocidad 200


float update_motors(float heading_error);
float control_output(float current_heading, float target_heading);
#endif // STABILIZATION_H_INCLUDED
