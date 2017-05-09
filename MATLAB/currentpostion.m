function [ newlat,newlong ] = currentpostion(course,lat,long,speed,time) 
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

distance =speed.*(0:1:time);
[newlat,newlong]=latgenerator(lat,long,distance,course);

end

