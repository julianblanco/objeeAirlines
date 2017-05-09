function [ dist ] = readDist( s,mode )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

%serial send read request to arduino
fprintf(s,mode);

%read value returned
dist = fscanf(s,'%f');


end

