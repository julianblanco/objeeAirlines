function [ value ] = fGetNumber( prompt,low,high )
%Function prompts user for a number and provides 
%data validation
%  The function

value= input(prompt);
while value > high || value < low
  if value <low
      disp('Error, Value too low. Please re-enter');
  
  elseif value >high
       disp('Error, Value too high. Please re-enter');
  else NaN(value)
       disp('Error, Value too low. Please re-enter');
  end%end if statment
  value= input(prompt);
end%end while loop
end%end fuction fGetNumber

