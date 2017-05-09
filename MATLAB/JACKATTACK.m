%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%JACK
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
comPort ='COM5';
BAUD = 9600;

if (~exist('serialFlag', 'var'))
    [arduino.s,serialFlag] = setupSerial(comPort,BAUD);
end

distance = readDist(arduino.s,'P');