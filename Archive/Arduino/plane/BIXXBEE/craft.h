//craft.h
#ifndef _CRAFT_H_
#define _CRAFT_H_


#define rollMaxAllowed 40
#define minAirspeed 15
#define maxAirspeed 30

void UpdateOrientation();
void Serial_Sample(void);

void Update_Control();
void notify_user();
float altitudeControl(float throttle, float altitude, float altitudeSetpoint, float airspeed, float desiredAirspeed);
//void UpdateServos();

void ControlResponse(float yawTargetValue, float rollTargetValue, float pitchTargetValue, float yawInput, float rollInput, float pitchInput) ;
float control_output(float current_heading, float target_heading);

float controlPID(float error,float Kp, float Ki, float Kd, int useKP, int useKI,int useKD ,float satuarationpoint,float satuarationPWMlow,float satuarationPWMhigh,float offset);

float saturate(float input,float upperbound,float lowerbound);


float crossTrackError(float distance2WP,float tracklegHead,float targetHead );
float crossTrackCorrection(float distanceXT,float targetHead );
//void CenterAll();

//void ServoInit();
#endif