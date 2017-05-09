clear all;
clc;

comPort = 'COM6';
BAUD=9600;
[arduino ,flag] = setupSerial(comPort,BAUD);
pause(.01);
dyFilt =0;
alpha = 5;
  dummy = fgetl(arduino);
for k =1:128;
%fprintf(arduino,'%c','P');
%pause(.05);
dist = fgetl(arduino);
dist = str2double(dist);
distance(k) = dist;
Output = fgetl(arduino);
Output = str2double(Output);
Outarray(k) = Output;
hold on
axis([0 40 0 200])
subplot(2,1,1)
plot(distance)
axis([0 120 0 200])
subplot(2,1,2)
plot(Outarray)

end


pause(.05);



pause(15)
closeSerial;