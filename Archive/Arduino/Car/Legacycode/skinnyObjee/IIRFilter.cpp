#include "IIRFilter.h"
#include <Arduino.h>

//function [steeringCompassOut, GL] = HeadingIIRFilter(GPSinput, steeringCompassIn, gyroCurrent, gyroLast,  inputPrecision)
//HEADINGIIRFILTER syncs the gyro realtive heading and gps true heading
//under as new steeringCompass heading. This will allow realtively accurate
//steering for some time after GPS fails. Run this recursively with self.
// -----------------------------------------------------------------------
// Parameters:
// 1: GPSinput --> this is the true heading from a gps sensor
// 2: steeringCompassIn --> the function is meant to be recursive, this is
//                          the output from the last run
// 3: gyroCurrent --> this is the relative heading from a gyro sensor
// 4: gyroLast --> last gyro heading from previous iteration of function
// 5: inputPrecision --> this is the decimal precision of the output heading
// ------------------------------------------------------------------------
// Output(s):
// 1: steeringCompassOut --> correlated output heading in degrees true
// 2: GL --> last gyro heading that will be fed into recursive function
// ------------------------------------------------------------------------
// Inifinite Impulse Response Filter
// ------------------------------------------------------------------------
// gyro runs 100Hz test at 75Hz, GPS runs at 1Hz test at 1Hz.
// Test gyro at 90 degrees, GPS at 0 degrees. Gyro needs to correct to GPS
// Start with precision of +-1 degree (epsilon value, ?ceiling fcn?)
// Start with 1 degree of freedom; to do this, fix the GPS value.
// Final product needs to work with changing GPS and Gyro; 2 variables
// -----------------------------------------------------------------------
// Important Note: the gyro and the GPS need to be synced computationally
// only, this will not involve movement of the vehicle.


// start of parameter shielding
float IIRFilter(float GPSin, float GC, float steeringCompass, float precision)
{
      //  static int LED = 0; // troubleshooting
	bool wrappingCounterClockwise = false; // True = shortest route is counterclockwise
	bool wrappingClockwise = false; // True = shortest route is clockwise
	float steeringCompassOut;
	float steeringCompassOriginal;
	float GPSoriginal;
	float epsilon = 1000;
	static float GL = 0;
	// if steeringCompass is 10 and GPS is 350 we need it to calculate the
	// shortest route possible eg: the 20degree difference across 360



	if ((GPSin - steeringCompass) > 180) // makes as decision if shortest route is counterclockwise
	{
		GPSin = 360 - GPSin + steeringCompass; // accounts for current steering compass
		steeringCompassOriginal = steeringCompass; // fixes inital steeringCourse
		wrappingCounterClockwise = true;// activates conditional in epsilon while loop
	}
	else if (GPSin - steeringCompass < -180) // makes as decision if shortest route is clockwise
	{
		steeringCompass = 360 - steeringCompass + GPSin; // accounts for current steering compass
		GPSoriginal = GPSin; // fixes inital steeringCourse
		wrappingClockwise = true;// activates conditional in epsilon while loop
	} // end if to determine wrapping clockwise, if any

//	while (epsilon >= precision)
//	{
	if (wrappingCounterClockwise == true)
	{
                //LED!=LED;
                //digitalWrite(13,LED);
		steeringCompass = steeringCompassRefactor(steeringCompass, GPSin, GL, GC);
		steeringCompassOut = steeringCompassOriginal - steeringCompass; // accounts for the original steering compass
		if (steeringCompassOut < 0)// wraps negative values from calculation
		{
			steeringCompassOut = steeringCompassOut + 360;
		}
	}// end if

	else if (wrappingClockwise == true)
	{
		steeringCompass = steeringCompassRefactor(steeringCompass, GPSin, GL, GC);
		steeringCompassOut = GPSin - steeringCompass + 360 + GPSoriginal; // accounts for the original GPS heading
		if (steeringCompassOut > 360)
		{ // wraps values over 360
			steeringCompassOut = steeringCompassOut - 360;
		}
	} // end else if
	else
	{
		steeringCompass = steeringCompassRefactor(steeringCompass, GPSin, GL, GC);
		steeringCompassOut = steeringCompass;
	} // end else

	//epsilon = abs(GPSin - steeringCompass); // precision calculation
	//} // end while epsilon >= .001

	GL = GC; // sets GC as last gyro heading for next iteration

	return steeringCompassOut;
} // end function


// how do i deal with no GPS input at start up?
// i need to start at a known state
// split combinedData and steering data so i can call them at different
// rates


float steeringCompassRefactor(float steeringCompass, float GPSin, float GL, float GC)
{
	float combinedData;

	int leftflag=0;
	int rightflag=0;
	

	combinedData = (1-GPS_COEF)*steeringCompass + GPS_COEF*GPSin; // weights values, narrows in on desired heading

	if ( (abs(GC-GL)) >180)
{
		if(GC>GL)
		{
			GC=GC-360;
		}
		if(GL>GC)
		{
			GL=GL-360;
		}
		if((GC -GL) <0)
		{
			leftflag=1;
		}
		if((GL-GC)<0)
		{
			rightflag=1;
		}
		if(leftflag==1)
		{
			steeringCompass = combinedData - (abs(GL) +abs(GC));
		}
		if(rightflag==1)
		{
			steeringCompass = combinedData +(abs(GL)+abs(GC));
		}
		steeringCompass =correct_wrap(steeringCompass);
}
	else
		{ steeringCompass=combinedData + (GC-GL);
			steeringCompass =correct_wrap(steeringCompass);
		}
{


}

	return steeringCompass;
}


  // Fix any 360 degree wrap around that may occur while updating angular position
float correct_wrap(float current_heading)
{
  // Correct 360 deg wrap around
  if ( current_heading > 360)
    current_heading = current_heading - 360;
  if (current_heading < 0)
    current_heading = current_heading + 360;
  return (current_heading);
}