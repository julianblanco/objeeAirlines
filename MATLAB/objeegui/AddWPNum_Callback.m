

function [NewLat, NewLong]=AddWPNum_Callback
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

NewWpBox =findobj(objeeAirlines,'Tag','textWP');

NewWP=get(NewWpBox,'String');
NewWP=str2double(NewWP);
assignin('base','NewWP',NewWP); 
assignin('base','MailBoxNewWPNum',1); 
end
