function [ dist ] = distance2waypoint( Lat1,Long1,Lat2,Long2 )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here


  dLat = (Lat2 - Lat1);                                  
  dLon = (Long2 - Long1) * cos(Lat1) ; 
  dist = sqrt((dLat*dLat) + (dLon*dLon)) * 110575;
  

end

