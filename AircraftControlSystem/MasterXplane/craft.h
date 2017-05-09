//craft.h
#ifndef _CRAFT_H_
#define _CRAFT_H_



#define minAirspeed 15
#define maxAirspeed 30

void UpdateOrientation();
void ControlResponseWing(float &yawTargetValue, float &rollTargetValue, float &pitchTargetValue, float yawInput, float rollInput, float pitchInput) ;
float controlPID(float ErrorCurrent, float Kp, float Ki, float Kd, int useKP, int useKI, int useKD, float satuarationpoint, float satuarationPWMlow, float satuarationPWMhigh, float offset, float &Integralresponse, float &ErrorOld,float inputsignal) ;
float saturate(float input,float upperbound,float lowerbound);

#endif