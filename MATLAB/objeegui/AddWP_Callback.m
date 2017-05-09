

function [NewLat, NewLong]=AddWP_Callback
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

NewLatBox =findobj(objeeAirlines,'Tag','NewLatbox');
NewLongBox =findobj(objeeAirlines,'Tag','NewLongbox');
NewLat=get(NewLatBox,'String');
NewLat=str2double(NewLat);
NewLong=get(NewLongBox,'String');
NewLong=str2double(NewLong);
assignin('base','NewLat',NewLat);
assignin('base','NewLong',NewLong); 
assignin('base','MailBoxNewWP',1); 
end
