//craft.h
#ifndef _CRAFTCONTROL_H_
#define _CRAFTCONTROL_H_


#define rollMaxAllowed 30
#define minAirspeed 15
#define maxAirspeed 30

void UpdateOrientation();
void controlResponseWing(float &yawTargetValue, float &rollTargetValue, float &pitchTargetValue, float yawIn, float rollIn, float pitchIn) ;
float controlPID(float ErrorCurrent, float Kp, float Ki, float Kd, int useKP, int useKI, int useKD, float satuarationpoint, float satuarationPWMlow, float satuarationPWMhigh, float offset, float &Integralresponse, float &ErrorOld,float inputsignal) ;
float saturate(float input,float upperbound,float lowerbound);

#endif