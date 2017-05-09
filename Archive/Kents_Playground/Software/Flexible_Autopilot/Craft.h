/*	Authors: Kent Altobelli & Caleb Stewart
	Date: 14Jan16
	
*/

#ifndef CRAFT_H
#define CRAFT_H

#include "Control_Surfaces.h"


// High level data interface for craft
class Craft
{
public:
	Craft();
	~Craft();

	// Call to update any sensors scheduled to give new data
	void update_data();

	void set_control_surface(int index, Control_Surface* servo);

	// Set variables (Assuming 3 element arrays for float points)
	void set_home(float* home);
	void set_orientation(float* des_orient);
	void set_rates(float* des_rates);
	void set_position(float* des_position);
	void set_airspeed(float des_airspeed);

	// Get variables (Gives pointer to first of 3 element array)
	void get_home(float* home) { return home; }
	void get_orientation(float* cur_orient) { return cur_orient; }
	void get_rates(float* cur_rates) { return cur_rates; }
	void get_position(float* cur_position) { return cur_position; }
	void get_airspeed(float cur_airspeed) { return cur_airspeed; }

private:
	// Airplane data
	string name;  // Name of aircraft
	static float home[3];  // Initialization point latitude, longitude and altitude (post-mission return point)
	static float cur_orient[3];  // Current roll, pitch and yaw orientation
	static float des_orient[3];  // Desired roll, pitch and yaw orientation
	static float cur_rates[3];  // Current roll, pitch and yaw turning/rolling rates
	static float des_rates[3];  // Desired roll, pitch and yaw turning/rolling rates
	static float cur_position[3];  // Current latitude, longitude and altitude
	static float des_position[3];  // Desired latitude, longitude and altitude
	static float cur_airspeed;  // Current airspeed (or other relevant speed)
	static float des_airspeed;  // Desired airspeed (or other relevant speed)
	static float xtrack_error;  // Current cross track error
	Control_Surface ctrl_surface[6];  // Control surfaces available on plane

	// Control system data
	float orient_P_coef[3];  // Orientation P coefficient for each axis
	//float orient_D_coef[3];  // Orientation D coefficient for each axis
	//float orient_I_coef[3];  // Orientation I coefficient for each axis
	//float orient_I_MAX_VALUE[3];  // Maximum allowable value for orientation integral windup
	float xtrack_P_coef[2];  // Crosstrack P coeffient for side-side and up-down
	
	// Constant values for specific platform
	float CRUISE_AIRSPEED;  // Ideal airspeed for efficient and responsive flight
	float STALL_AIRSPEED;  // Airspeed where plane will stall
	float MAX_ANGLE[2];  // Maximum roll and pitch angles
	float MAX_RATE[3];  // Maximum roll, pitch and yaw rates
	int NUM_CTRL_SURFACES;  // Number of control surfaces available


	// Sensor Objects
	GPS m_gps;
	IMU m_imu;
	AirSpeed m_air;
	Barometer m_bar;

	// Decide which sensors to poll to hit poll rates
	void poll_sensors();

	// Update high level data members after polling relevant sensor
	void update_orientation();
	void update_position();
	void update_airspeed();
};


#endif  //CRAFT_H