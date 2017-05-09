#include "IIRFilter.h"
#include <Arduino.h>
/*
//===================================================================
//
//	Program:		Navigation.cpp
//
//	Author:			Julian Blanco
//	Date:			Sep 4 2015
//
//	Description:	Description
//
//===================================================================

PAT

Things you might need. The global variables for the data are

float trueHeading
float GpsHeading
float lastEuler=0;
float currentEuler=0;

if you need to add global variables I store all of the variables in varaibles.h
try not to declare anything inside of headers and cpp files

if you do need to declare a interger counter or something inside of a function and you dont want to use a variable you can use

static int i=0;

it wont be acessable anywhere except the function the static makes it retain its value between loops







function [steeringCompassOut, GL] = HeadingIIRFilter(GPSinput, steeringCompassIn, gyroCurrent, gyroLast,  inputPrecision)
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
*/
float IIRFilter(float GPSin, float GC, float steeringCompass, float precision)
{
        static int LED = 0; // troubleshooting
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
                LED!=LED;
                digitalWrite(13,LED);
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

	epsilon = abs(GPSin - steeringCompass); // precision calculation
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

	combinedData = .5*steeringCompass + .5*GPSin; // weights values, narrows in on desired heading
	steeringCompass = combinedData + (GL - GC); // steering compass

	return steeringCompass;
}

