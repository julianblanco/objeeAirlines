% convert gps (lat & long) coordinates to utm coordinates (north & east)
% 1/c Ledzian, Patrick
% 22 Jan 17

lat1 = 41.402537;
long1 = -71.883863;

lat2 = 41.403023;
long2 = -71.884072;

zone = 19;       % CT and Mass are on the line 18/19

[ x,y ] = gps2xy_not_accurate( lat1, long1, zone );  % the result is not correct, not sure why
