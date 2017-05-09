// Kent Altobelli
// 02 March 2016
// Uses analog joystick to calculate the compass heading of the joystick position

// Verical direction measured by Analog 0
// Horizontal direction measured by Analog 1



#include <Arduino.h>
#include <math.h>

// 1 or -1
#define FLIP_VERT 1  // Vertical on Analog Channel 0
#define FLIP_HOR -1  // Horizontal on Analog Channel 1
#define pi 3.14159f


int vert_val = 0;
int hor_val = 0;
int curr_head = 0;

void setup()
{
	Serial.begin(9600);
}

void loop()
{
	// Read analog inputs and shift to go from -512 to 512
	vert_val = FLIP_VERT * (analogRead(0) - 512);
	hor_val = FLIP_HOR * (analogRead(1) - 512);

	// Convert to magnitude and angle
	int mag = (int)sqrt(pow((float)vert_val,2)+pow((float)hor_val,2));
	int angle = 360 - wrap_head((int)(180/pi) * atan2((float)vert_val,(float)hor_val) - 90);
  
	// Use new angle only if magnitude is large enough (small deflections aren't used)
	if( mag > 256 )
	{
		curr_head = angle;
	}

	// Output heading to serial
	Serial.println(curr_head);
}



int wrap_head(int angle)
{
  if( angle > 360 ) angle-=360;
  if( angle < 0 ) angle+=360;

  return(angle);
}

