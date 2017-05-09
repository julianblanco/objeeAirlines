clear all;clc

csaileron=0; csrudder=0; cselevator=0;

%airleron (roll) is in x axis
%rudder (yaw) is in y axis
%elevator(pitch) is in z axis

currentYaw=0;
currentRoll=0;
currentPitch=0;
% offsetYaw=0-currentYaw;
% offsetRoll=0-currentRoll;
% offsetPitch=0-currentPitch;

desiredYaw=180; desiredRoll =0; desiredPitch =0;


disp('The Current Yaw is')
disp(currentYaw)
disp('The Current Roll is')
disp(currentRoll)
disp('The Current Pitch is')
disp(currentPitch)


disp('The Desired Yaw is')
disp(desiredYaw)
disp('The Desired Roll is')
disp(desiredRoll)
disp('The Desired Pitch is')
disp(desiredPitch)

yawPcoef =20;rollPcoef=20;pitchPcoef=20;

currentYaw=(pi/180)*(currentYaw);
currentRoll=(pi/180)*(currentRoll);
currentPitch=(pi/180)*(currentPitch);
desiredYaw=(pi/180)*(desiredYaw);
desiredRoll=(pi/180)*(desiredRoll);
desiredPitch=(pi/180)*(desiredPitch);
% offsetYaw=(pi/180)*(offsetYaw);
% offsetRoll=(pi/180)*(offsetRoll);
% offsetPitch=(pi/180)*(offsetPitch);

yawError=currentYaw - desiredYaw;
rollError=currentRoll - desiredRoll;
pitchError=currentPitch - desiredPitch;

yawResponse=yawError*yawPcoef;
pitchResponse=pitchError*pitchPcoef;
rollResponse=rollError*rollPcoef;


csrudder=cos(currentRoll)*yawResponse + sin(currentRoll)*pitchResponse;
cselevator=sin(currentRoll)*yawResponse + cos(currentRoll)*pitchResponse;
csaileron=rollResponse;

disp('The Aileron Response is')
disp(csaileron)
disp('The Elevator Response is')
disp(cselevator)
disp('The Rudder Response is')
disp(csrudder)




