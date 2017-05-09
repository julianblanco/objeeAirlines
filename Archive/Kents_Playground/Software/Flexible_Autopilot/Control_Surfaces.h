/*
	Kent Altobelli
*/

#ifndef CONTROL_SURFACES_H
#define CONTROL_SURFACES_H


// Enumeration to keep axes straight throughout program (yaw always referenced to true north)
enum axis { ROLL = 0, PITCH, YAW };

// Class for information regarding each control surface
class Control_Surface {
public:
	Control_Surface(string name, Servo servo, int pin, int center, int min, int max);
	~Control_Surface();

	string name;  // Name of control surface
	int pin;  // Servo signal pin
	Servo servo;  // Servo object
	int center;  // Servo center point (us)
	int min;  // Servo minimum signal (us)
	int max;  // Servo maximum signal (us)
	float axis_weight[3];  // Effect of specific control surface on roll, pitch and yaw (add to 1)
};


#endif