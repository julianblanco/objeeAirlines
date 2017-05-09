//craft.h
#ifndef _CRAFT_H_
#define _CRAFT_H_


#define rollMaxAllowed 30

void UpdateOrientation();
void Serial_Sample(void);

void Update_Control();
void notify_user();
void waypointing();
//void UpdateServos();

void ControlResponse(float yawTargetValue, float rollTargetValue, float pitchTargetValue, float yawInput, float rollInput, float pitchInput) ;
float control_output(float current_heading, float target_heading);

float rollPID(float error) ;
float yawPID(float error) ;
float pitchPID(float error);
float saturate(float input,float upperbound,float lowerbound);


float crossTrackError(float distance2WP,float tracklegHead,float targetHead );
float crossTrackCorrection(float distanceXT,float targetHead );
//void CenterAll();

//void ServoInit();
#endif