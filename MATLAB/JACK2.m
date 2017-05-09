clear all;
clc;

comPort = 'COM5';
BAUD=9600;
[arduino ,flag] = setupSerial(comPort,BAUD);
pause(.01);
Lidar=findobj('Tag','Lidar');
loop=get(Lidar,'UserData');
while loop ~=  10;
  while loop == 1
    fprintf(arduino,'%c','P');
    pause(.1);
    dist = fgetl(arduino);
    dist = str2double(dist);
    distance(k) = dist;

    subplot(4,1,1)
    hold on
    plot(distance)
    axis([(k-50) (k + 100) (0) (dist+150)] )
    loop = get (Lidar,'UserData');
    pause(.1);
  end
end



closeSerial;