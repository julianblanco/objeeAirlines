clear all
clc
% x=[ 458731; ];
% y=[4462881;];
% utmzone=['30 T'; ];
% [Lat, Lon]=utm2deg(x,y,utmzone);
% fprintf('%11.6f ',Lat)
% fprintf('%11.6f ',Lon)
%    
A1= 6378137
F1=298.2572

xs = 710453
ys=4576823
f = 18%zone

[lat ,long]=utm2ll(xs,ys,f)

D0 = 180/pi;	% conversion rad to deg
maxiter = 100;	% maximum iteration for latitude computation
eps = 1e-11;	% minimum residue for latitude computation

K0 = 0.9996;					% UTM scale factor
X0 = 500000;					% UTM false East (m)
Y0 = 1e7*(f < 0);				% UTM false North (m)
P0 = 0;						% UTM origin latitude (rad)
L0 = (6*abs(f) - 183)/D0;			% UTM origin longitude (rad)
E1 = sqrt((A1^2 - (A1*(1 - 1/F1))^2)/A1^2);	% ellpsoid excentricity
N = K0*A1;



c0 = [-175/16384, 0,   -5/256, 0,  -3/64, 0, -1/4, 0, 1;
           -105/4096, 0, -45/1024, 0,  -3/32, 0, -3/8, 0, 0;
           525/16384, 0,  45/1024, 0, 15/256, 0,    0, 0, 0;
          -175/12288, 0, -35/3072, 0,      0, 0,    0, 0, 0;
          315/131072, 0,        0, 0,      0, 0,    0, 0, 0];
for i = 1:size(c0,1)
    c(i) = polyval(c0(i,:),E1);
end   
C=c;

YS = Y0 - N*(C(1)*P0 + C(2)*sin(2*P0) + C(3)*sin(4*P0) + C(4)*sin(6*P0) + C(5)*sin(8*P0));

c0 = [-175/16384, 0,   -5/256, 0,  -3/64, 0, -1/4, 0, 1;
             1/61440, 0,   7/2048, 0,   1/48, 0,  1/8, 0, 0;
          559/368640, 0,   3/1280, 0,  1/768, 0,    0, 0, 0;
          283/430080, 0, 17/30720, 0,      0, 0,    0, 0, 0;
       4397/41287680, 0,        0, 0,      0, 0,    0, 0, 0];
   
for i = 1:size(c0,1)
    c(i) = polyval(c0(i,:),E1);
end

C=c;

zt = complex((ys - YS)/N/C(1),(xs - X0)/N/C(1));
z = zt - C(2)*sin(2*zt) - C(3)*sin(4*zt) - C(4)*sin(6*zt) - C(5)*sin(8*zt);
L = real(z);
LS = imag(z);

l = L0 + atan(sinh(LS)./cos(L));
p = asin(sin(L)./cosh(LS));

L = log(tan(pi/4 + p/2));

% calculates latitude from the isometric latitude
p = 2*atan(exp(L)) - pi/2;
p0 = NaN;
n = 0;
while any(isnan(p0) | abs(p - p0) > eps) && n < maxiter
	p0 = p;
	es = E1*sin(p0);
	p = 2*atan(((1 + es)./(1 - es)).^(E1/2).*exp(L)) - pi/2;
	n = n + 1;
end

	lat = p*D0
	lon = l*D0