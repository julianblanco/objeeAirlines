//craft.h
#ifndef _TACTICS_H_
#define _TACTICS_H_


#define rollMaxAllowed 30
#define minAirspeed 15
#define maxAirspeed 30
void controlTactics(void);
void altitudeControl(float& throttle,float &desiredPitch, float altitude, float altitudeSetpoint, float airspeed, float desiredAirspeed);
float crossTrackError(float distance2WP,float tracklegHead,float targetHead );
float crossTrackCorrection(float distanceXT,float targetHead,float distance2WP);
float correct_wrap(float current_heading);
// Find the shortest signed angular difference between a target and source angle
float angular_diff(float target_angle, float source_angle);

#endif