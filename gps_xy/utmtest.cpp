A1 = 6378137;
F1 = 298.2572;

xs = 710453;
ys=4576823;

f = 18;

D0 = 180 /3.14159;
maxiter = 100;
eps = 0.000000000001;
k0 = .9996;
X0 = 500000;
Y0 = 10000000;
if (f <0) Y0=0;
P0 = 0;
L0 = (6 *abs(f) -183)/D0;
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












	