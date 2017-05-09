altitudeError =5;%meters
headingError =10;%degrees
pitchError=5;
rollerror=5;

roll=0;
distance2wp=100;%meters
targetheading=100;

rollmax=30;
pitchmax=8;

altitudeKP = 1.5;
rollkp=9;
pitchkp=4.5;
yawkp=1;



%Crosstrack error
distanceXT = distance2wp * sin(headingError);
%%
%Crosstrack Correction
xtCoeff =-100;
crabangle=(xtCoeff*distanceXT)/distance2wp;
if crabangle > 30
    crabangle=30;
end
if crabangle < -30
    crabangle=-30;
end

targetheading = targetheading + crabangle;
%%
%%Roll Calculation
rollangledesired = headingerror*yawkp;

if rollangledesired > rollmax
    rollangledesired = rollmax;
end
if rollangledesired < -rollmax
    rollangledesired = -rollmax;
end
%%
%%Pitch Calculation
desiredpitch = altitudeerror * altitudeKP;


if desiredpitch > pitchmax
    desiredpitch = pitchmax;
end
if desiredpitch < -pitchmax
    desiredpitch = -pitchmax;
end
%%
%%Servo calcs

rollservoout = rollkp*rollerror;
pitchservoout = pitchkp*pitcherror;
yawservoout = yawkp*yawerror;

blendedpitchservo = (sin(-1*roll)*yawservoout)+(cos(roll)*pitchServoOut)

leftservo = blendedpitchservo - rollservoout*2 +1500;
rightservo = -1*blendedpitchservo - rollservoout*2 +1500;