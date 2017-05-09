#include "Control_Sufaces.h"
#include "ServoM.h"


// Constructor for all servo stuff
Control_Surface::Control_Surface(string name, Servo servo, int pin, int center, int min, int max) : string(name), Servo(servo), int(pin), int(center), int(min), int(max);
{
	axis_weight[ROLL] = roll_weight;
	axis_weight[PITCH] = pitch_weight;
	axis_weight[YAW] = yaw_weight;
}


// Destructor
Control_Surface::~Control_Surface() {}