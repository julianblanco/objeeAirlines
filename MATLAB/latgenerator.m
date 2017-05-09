function [ newlat,newlong ] = latgenerator( currentlat,currentlong,distance,course )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

degrees = distance/111319;
course=degtorad(course);
lat=degrees*cos(course);
long=degrees*sin(course);

newlat=currentlat+lat;
newlong=currentlong+long;

end

