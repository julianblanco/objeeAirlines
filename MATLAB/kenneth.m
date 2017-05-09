clear all;
clc;

comPort = 'COM6';
BAUD=9600;
[arduino ,flag] = setupSerial(comPort,BAUD);
pause(.01);
dyFilt =0;
alpha = 5;
  
for k =1:128;
%fprintf(arduino,'%c','P');
%pause(.05);
dist = fgetl(arduino);
dist = str2double(dist);
distance(k) = dist;
hold on
plot(distance)

axis([0 130 0 500])
end


pause(.05);



pause(5)
closeSerial;