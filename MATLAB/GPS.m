clear all
clc
clf

gpsdata= csvread('MATLAB.txt');
%lat = gpsdata(1:5,1);
%long= gpsdata(1:5,2);

lat = num2str(gpsdata(1:end,1));
long= num2str(gpsdata(1:end,2));
lat=lat(:,1:2)
long=long(:,1:2)
lat1=str2num(lat)
long1=str2double(long)
lat2=str2double(lat(:,3:8))./60
long2=str2double(long(:,3:8))./60
lat3=lat1+lat2
long3=long2+long1
% 
% 
% wplat = [41.372518,41.372518, 41.372236, 41.372477, 41.372518 ];
% wplong = [-72.099085,-72.099085, -72.098959, -72.098802, -72.099085 ];%axis([-72.0993 -72.0987 41.3721 41.3728])
% avg=0;
scatter(long3(1:260),lat3(1:260));
% for i=1:1:200
%  dist = distance2waypoint(lat(i),long(i),4.122326900000001e+03,7.205978500000000e+03)
%  avg =(dist+avg)/i;
% figure(2)
% scatter(long((i)),lat((i)),'b')
% hold on
% scatter(long(i),lat(i),'r')
% scatter( -72.098684,41.372691,'g')
% pause(.0000001)
% end
% %plot(wplong(:),wplat(:))
% %plot(wplong(1:end),wplat(1:end))
% 
