clear all
% x=[ 458731; ];
% y=[4462881;];
% utmzone=['30 T'; ];
% [Lat, Lon]=utm2deg(x,y,utmzone);
% fprintf('%11.6f ',Lat)
% fprintf('%11.6f ',Lon)
%    
lat = 41.315430;
long=-72.485713;
[xs1 ,ys1]=ll2utm(lat  ,long)
datum=6378137;
datumf1=298.2572;
K0 = 0.9996;
X0 = 500000;

phi = lat /(180/pi);
lamda = long /(180/pi);
F0 = round((lamda*(180/pi) + 183)/6);
N = K0*datum;

B1 = datum*(1 - 1/datumf1);
E1 = sqrt((datum*datum - B1*B1)/(datum*datum));

c0 = [-175/16384, 0,   -5/256, 0,  -3/64, 0, -1/4, 0, 1;
         -901/184320, 0,  -9/1024, 0,  -1/96, 0,  1/8, 0, 0;
         -311/737280, 0,  17/5120, 0, 13/768, 0,    0, 0, 0;
          899/430080, 0, 61/15360, 0,      0, 0,    0, 0, 0;
      49561/41287680, 0,        0, 0,      0, 0,    0, 0, 0];
  
c = zeros(size(c0,1),1);
P0=0;

for i = 1:size(c0,1)
    c(i) = polyval(c0(i,:),E1);
end 

C =c;
  
B = C(1)*P0 + C(2)*sin(2*P0) + C(3)*sin(4*P0) + C(4)*sin(6*P0) + C(5)*sin(8*P0);
YS = 0 - N*B;

L0 = (6*F0 - 183)/(180/pi);
L = log(tan(pi/4 + phi/2).*(((1 - E1*sin(phi))./(1 + E1*sin(phi))).^(E1/2)));
z = complex(atan(sinh(L)./cos(lamda - L0)),log(tan(pi/4 + asin(sin(lamda - L0)./cosh(L))/2)));
Z = N.*C(1).*z + N.*(C(2)*sin(2*z) + C(3)*sin(4*z) + C(4)*sin(6*z) + C(5)*sin(8*z));
xs = imag(Z) + X0;
ys = real(Z) + YS;

xs1-xs
ys1-ys
%%


