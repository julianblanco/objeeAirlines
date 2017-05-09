function [ course ] = course2waypoint( lat1,long1,lat2,long2 )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here


 dlon = degtorad((long2 - long1));
  lat1 = degtorad(lat1);
  lat2 = degtorad(lat2);
  a1 = sin(dlon) * cos(lat2);
  a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a3 = cos(lat1) * sin(lat2) - a2;
  a4 = atan2(a1, a3);
  if a4< 0.0
    a4 = a4+2*pi;
  end
 course= todegrees(a4);

end

