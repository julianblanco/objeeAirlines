************************************************************
clear all; hold off;clf;
w=logspace(-1,3,100);
% generate 100 points equally spaced on a logarithmic axis
% from 0.1 to 1000
b=[-0.4 20];
%numerator = -.4s + 20
a=[1 2];
%denominator = 1s + 2
h=freqs(b,a,w);
%evaluate frequency response at those frequencies in the w vector
mag=20*log10(abs(h));
phase=angle(h)*180/pi;
subplot(2,1,1);
semilogx(w,mag);
xlabel('Frequency in radians/sec');
ylabel('Gain in dB');grid;
subplot(2,1,2);
semilogx(w,phase);
xlabel('Frequency in radians/sec');
ylabel('Phase in Degrees');grid;
************************************************************