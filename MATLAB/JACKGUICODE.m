clear all;
clc;

comPort = 'COM6';
BAUD=9600;

[arduino ,flag] = setupSerial(comPort,BAUD);
pause(.01);

Lidar=findobj('Tag','box');
Distbox =findobj('Tag','dist');


set(Lidar,'String','On');
loop=get(Lidar,'String');
loop
k=0;
%while loop ~=  '10';
    
    loop
  while strcmp(loop, 'On')
    k=k+1;
    fprintf(arduino,'%c','P');
    pause(.1);
    dist = fgetl(arduino);
    dist = str2double(dist);
    distance(k) = dist;

   
 
 % plot(distance)
  %set(handles.axes5,'Visible', 'on', 'Units', 'pixels');
    set(Distbox,'String', dist);
    hold on
    plot(distance)
    axis([(k-50) (k + 100) (0) (dist+350)] )
    loop = get (Lidar,'String');
    pause(.1);
  end
  loop = get (Lidar,'String');
%end


