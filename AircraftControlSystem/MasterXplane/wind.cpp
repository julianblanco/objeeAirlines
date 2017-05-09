#include "wind.h"
#include "variables.h"
#include "Navigation.h"
//===================================================================
//
//  Program:    Navigation.cpp
//
//  Author:     Julian Blanco
//  Date:       Oct 24 2015
//
//  Description:  Description
//
//===================================================================


void wind_estimation(void){

	static float x_component_sog_old = 0;
	static float y_component_sog_old = 0;
	static float z_component_sog_old = 0;

	static float x_component_air_old = 0;
	static float y_component_air_old = 0;
	static float z_component_air_old = 0;


	//calculate speed over ground relative to earth
	float x_component_sog = GpsSpeed*cos(degtorad(Gpsheading));
	float y_component_sog = GpsSpeed*sin(degtorad(Gpsheading));
	float z_component_sog = GpsSpeed*sin(degtorad(pitchInput));

	float x_component_air = AirSpeed*cos(degtorad(trueHeading));
	float y_component_air = AirSpeed*sin(degtorad(trueHeading));
	float z_component_air = AirSpeed*sin(degtorad(pitchInput));

	float x_component_wind = 0.5*((x_component_sog + x_component_sog_old) - (x_component_air + x_component_air_old));
	float y_component_wind = 0.5*((y_component_sog + y_component_sog_old) - (y_component_air + y_component_air_old));
	float z_component_wind = 0.5*((z_component_sog + z_component_sog_old) - (z_component_air + z_component_air_old));

	winddir = ((atan(y_component_wind/x_component_wind))*(180/3.14159265359));
	windmag = sqrt((x_component_wind*x_component_wind)+(y_component_wind*y_component_wind));//+(z_component_wind*z_component_wind) );

	x_component_sog_old = x_component_sog;
	y_component_sog_old = y_component_sog;
	z_component_sog_old = z_component_sog;

	x_component_air_old = x_component_air;
	y_component_air_old = y_component_air;
	z_component_air_old = z_component_air;

}
