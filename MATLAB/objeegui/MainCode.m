clear all;
clf;
clc;
closeSerial;
try
    comPort = '/dev/ttyUSB0';
    BAUD=9600;
    [xbee ,flag] = setupSerial(comPort,BAUD);
catch
    try 
        comPort = '/dev/ttyUSB1';
        BAUD=9600;
        [xbee ,flag] = setupSerial(comPort,BAUD);
    catch
        comPort = '/dev/ttyUSB2';
         BAUD=9600;
        [xbee ,flag] = setupSerial(comPort,BAUD);
    end
end


Targetbox =findobj(objeeAirlines,'Tag','Target');
Truebox =findobj(objeeAirlines,'Tag','True');
GPSHeadingbox =findobj(objeeAirlines,'Tag','GpsHeading');
Gyrobox =findobj(objeeAirlines,'Tag','Gyro');
HeadingErrorbox =findobj(objeeAirlines,'Tag','HeadingError');
Fixbox =findobj(objeeAirlines,'Tag','Fix');
WPnumbox =findobj(objeeAirlines,'Tag','WPnumbox');
Speedbox =findobj(objeeAirlines,'Tag','SOG');
CurLat =findobj(objeeAirlines,'Tag','CurrentLat');
CurLong =findobj(objeeAirlines,'Tag','CurrentLong');
TarLat =findobj(objeeAirlines,'Tag','TargetLat');
TarLong =findobj(objeeAirlines,'Tag','TargetLong');
map=findobj(objeeAirlines,'Tag','map');
alt=findobj(objeeAirlines,'Tag','Altbox');
armedbox=findobj(objeeAirlines,'Tag','ArmBox');

%axes(map);


x=[.371479, .373211];
y=[.099722, .098515];

matlabImage = imread('football3.png');
image(x,y,matlabImage,'Parent',map);   

pause(.01);
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
TargetLat=0;
TargetLong=0;
SpeedOverGround=0;
wpnum=0;

MailBoxNewWP=0;
MailBoxNewWPNum=0;
MailBoxArm=0;
% Armed=0;
% disp('hi')
% fprintf(xbee,'%c','$OA002\n');
% pause(1)
% jake=41;
% while jake >10 || jake <1
% dist= fgetl(xbee)
% c = strsplit(dist,',');
% jake=str2double(c(1))
% pause(1)
% end
% wplat=[];
% wplong=[];
% counter=1;
% for x=1:jake+1
%     counter=counter+1;
%     wplat(x)=str2double(c(counter));
%     counter=counter+1;
%     wplong(x)=str2double(c(counter));
% end
% disp('bye')
% for x=1:jake+1
%     targetLat=(wplat(x)-41)
%     targetLong=-1*(wplong(x)+72)
%    o=scatter(targetLat,targetLong,'b','filled');  %# Plot some random data
%    set(o,'Parent',map);
%    drawnow;
%    
% end

for k =1:3000;


 try
    dist= fgetl(xbee);
    c = strsplit(dist,',');

    set(Targetbox,'String',c(5));
    set(Truebox,'String',c(6));
    set(GPSHeadingbox,'String',c(6));
    set(Gyrobox,'String',c(8));
    set(HeadingErrorbox,'String',num2str(HeadingError));
    set(Fixbox,'String',c(9));
    set(CurLat,'String',c(1));
    set(CurLong,'String',c(2));
    set(TarLat,'String',c(3));
    set(TarLong,'String',c(4));
    set(Speedbox,'String',c(7));
    set(WPnumbox,'String',c(8));
    set(alt,'String',c(10));
    Armed=str2double(c(13));
    set(armedbox,'String',c(13)),
    pause(.001)
 catch
     dist='';
     dist= fgetl(xbee);
     dist='';
     dist= fgetl(xbee);
     c = strsplit(dist,',');

    set(Targetbox,'String',c(5));
    set(Truebox,'String',c(6));
    set(GPSHeadingbox,'String',c(6));
    set(Gyrobox,'String',c(8));
    set(HeadingErrorbox,'String',num2str(HeadingError));
    set(Fixbox,'String',c(9));
    set(CurLat,'String',c(1));
    set(CurLong,'String',c(2));
    set(TarLat,'String',c(3));
    set(TarLong,'String',c(4));
    set(Speedbox,'String',c(7));
    set(WPnumbox,'String',c(8));
    set(alt,'String',c(10));
    Armed=str2double(c(13));
    set(armedbox,'String',c(13)),
    pause(.001)
 end
    

 
    CurrentLat = str2double(c(1))-41;
    CurrentLong = str2double(c(2))+72;
    targetLat=str2double(c(3))-41;
    targetLong = str2double(c(4))+72; 
    j=scatter(CurrentLat,CurrentLong,'b','filled');  %# Plot some random data
     set(j,'Parent',map);
    o=scatter(targetLat,targetLong,'p','h');  %# Plot some random data
     set(o,'Parent',map);
   
    drawnow;
    
    if MailBoxNewWP
       msg=strcat('$OA001,',num2str(NewLat),',',num2str(NewLong),'\n');
       fprintf(xbee,'%c',msg);
       MailBoxNewWP=0;
    end
   
    if MailBoxNewWPNum
       msg=strcat('$OA003,',num2str(NewWP),'\n');
       fprintf(xbee,'%c',msg);
       MailBoxNewWPNum=0;
    end
    if MailBoxArm
        if  Armed
            Armed=0;
        else
            Armed=1;
        end
        
       msg=strcat('$OA005,',num2str(Armed),'\n');
       disp(msg)
       fprintf(xbee,'%c',msg);
       MailBoxArm=0;
    end
end
    

closeSerial()
   disp('Finished')
   
   hold off;
   clf;
