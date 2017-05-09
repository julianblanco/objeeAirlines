clear all;
clc;
closeSerial;
comPort = '/dev/ttyUSB0';
BAUD=9600;



Targetbox =findobj(Bobogui,'Tag','Target');
Truebox =findobj(Bobogui,'Tag','True');
GPSHeadingbox =findobj(Bobogui,'Tag','GpsHeading');
Gyrobox =findobj(Bobogui,'Tag','Gyro');
HeadingErrorbox =findobj(Bobogui,'Tag','HeadingError');
Fixbox =findobj(Bobogui,'Tag','Fix');
WPnumbox =findobj(Bobogui,'Tag','WPnumbox');
Speedbox =findobj(Bobogui,'Tag','SOG');
CurLat =findobj(Bobogui,'Tag','CurrentLat');
CurLong =findobj(Bobogui,'Tag','CurrentLong');
TarLat =findobj(Bobogui,'Tag','TargetLat');
TarLong =findobj(Bobogui,'Tag','TargetLong');
map=findobj(



[arduino ,flag] = setupSerial(comPort,BAUD);
pause(.01);
dummy = fgetl(arduino);
targetHeadingArray = {};
trueHeadingArray = {};
gpsHeadingArray = {};
gyroHeadingArray = {};
timeArray = {};
Hour = ' ' ;
Minute = ' ';
Second = ' ';

TargetHeading=0;
TrueHeading=0;
GpsHeading=0;
GyroHeading=0;
HeadingError=0;
Fix=0;
CurrentLat=0;
CurrentLong=0;
TargetLat=0;
TargetLong=0;
SpeedOverGround=0;
wpnum=0;

axes(map)
matlabImage = imread('football.jpg');
image(matlabImage)
axis off
axis image

for k =1:3000;
dist= fgetl(arduino);
display(dist)



c = strsplit(dist,',');



    




    set(Targetbox,'String',c(2));
    set(Truebox,'String',c(3));
    set(GPSHeadingbox,'String',c(4));
    set(Gyrobox,'String',num2str(GyroHeading));
    set(HeadingErrorbox,'String',num2str(HeadingError));
    set(Fixbox,'String',num2str(Fix));
    set(CurLat,'String',num2str(CurrentLat));
    set(CurLong,'String',num2str(CurrentLong));
    set(TarLat,'String',num2str(TargetLat));
    set(TarLong,'String',num2str(TargetLong));
    set(Speedbox,'String',num2str(SpeedOverGround));
    set(WPnumbox,'String',c(1));
   
    pause(.001)
    
    drawnow;
    
end
    


   disp('Finished')
