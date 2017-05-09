#ifndef _PID_H_
#define _PID_H_





float rollPID(float error,int rollKp,int rollKi, int rollKd, float timed,int derivativeSetpoint);

float pitchPID(float error,int rollKp,int rollKi, int rollKd,float timed,int derivativeSetpoint);

float yawPID(float error,int yawKp,int yawKi, int yawKd,float timed,int derivativeSetpoint);

#endif // STABILIZATION_H_INCLUDED
