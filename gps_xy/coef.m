%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function c = coef(e,m)
%COEF Projection coefficients
%	COEF(E,M) returns a vector of 5 coefficients from:
%		E = first ellipsoid excentricity
%		M = 0 for transverse mercator
%		M = 1 for transverse mercator reverse coefficients
%		M = 2 for merdian arc


if nargin < 2
	m = 0;
end

switch m
	case 0
	c0 = [-175/16384, 0,   -5/256, 0,  -3/64, 0, -1/4, 0, 1;
           -105/4096, 0, -45/1024, 0,  -3/32, 0, -3/8, 0, 0;
           525/16384, 0,  45/1024, 0, 15/256, 0,    0, 0, 0;
          -175/12288, 0, -35/3072, 0,      0, 0,    0, 0, 0;
          315/131072, 0,        0, 0,      0, 0,    0, 0, 0];
	  
	case 1
	c0 = [-175/16384, 0,   -5/256, 0,  -3/64, 0, -1/4, 0, 1;
             1/61440, 0,   7/2048, 0,   1/48, 0,  1/8, 0, 0;
          559/368640, 0,   3/1280, 0,  1/768, 0,    0, 0, 0;
          283/430080, 0, 17/30720, 0,      0, 0,    0, 0, 0;
       4397/41287680, 0,        0, 0,      0, 0,    0, 0, 0];

	case 2
	c0 = [-175/16384, 0,   -5/256, 0,  -3/64, 0, -1/4, 0, 1;
         -901/184320, 0,  -9/1024, 0,  -1/96, 0,  1/8, 0, 0;
         -311/737280, 0,  17/5120, 0, 13/768, 0,    0, 0, 0;
          899/430080, 0, 61/15360, 0,      0, 0,    0, 0, 0;
      49561/41287680, 0,        0, 0,      0, 0,    0, 0, 0];
   
end
c = zeros(size(c0,1),1);

for i = 1:size(c0,1)
    c(i) = polyval(c0(i,:),e);
end
end