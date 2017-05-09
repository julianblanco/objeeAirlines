function [ x_utm,y_utm ] = gps2xy( lat, lon, zone )
%GPS2XY convery GPS coordinates to xy coordinates using Mercator Projection
%   Check out the UTM zone map to determine what zone you are in

% parameters and functions for UTM<->:aw/Long conversions
RADIUS = 6378137.0;
FLATTENING = 0.00335281068;     % GRS80 or WGS84
K_NOT = 0.9996;                 % UTM scale factor
DEGREES_TO_RADIANS = 0.01745329252;
FALSE_EASTING = 500000.0;
FALSE_NORTHING = 10000000.0;

% necessary geodetic parameters and constants
lambda_not = deg2rad(((-180.0 + zone*6.0) - 3.0));% / RADIUS;  % RADIANS?
e_squared = 2.0*FLATTENING - FLATTENING^2;
e_fourth = e_squared^2;
e_sixth = e_squared^3;
e_prime_sq = e_squared/(1.0 - e_squared);
sin_phi = sin(lat);
tan_phi = tan(lat);
cos_phi = cos(lat);
N = RADIUS/sqrt(1.0 - e_squared*sin_phi^2);
T = tan_phi^2;
C = e_prime_sq*cos_phi^2;
M = RADIUS*((1.0 - e_squared*0.25 - 0.046875*e_fourth - 0.01953125*e_sixth)*lat - (0.375*e_squared + 0.09375*e_fourth + 0.043945313*e_sixth)*sin(2.0*lat) + (0.05859375*e_fourth + 0.043945313*e_sixth)*sin(4.0*lat) - (0.011393229 * e_sixth)*sin(6.0*lat));
A = (lon-lambda_not)*cos_phi;
A_sq = A^2;
A_fourth = A^4;

% now computer X and Y
x_utm = K_NOT*N*(A + (1.0 - T + C)*A_sq*A/6.0 + (5.0 - 18.0*T + T*T + 72.0*C - 58.0*e_prime_sq)*A_fourth*A/120.0);
y_utm = K_NOT*(M + N*tan_phi*(A_sq/2.0 + (5.0 - T + 9.0*C + 4.0*C*C)*A_fourth/24.0 + (61.0 - 58.0*T + T*T + 600.0*C - 330.0*e_prime_sq)*A_fourth*A_sq/720.0));

% correct for false easting and northing
if lat < 0
    y_utm = y_utm + 10000000.0;
else
    x_utm = x_utm + 500000;  

end

