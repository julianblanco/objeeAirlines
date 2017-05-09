for r=1:10
    
    
    
     %Update current heading given the change in the gyro angle since last polled
  current_heading = current_heading + (last_euler - current_euler);
  %// update state estimation

  % Correct 360 deg wrap around
  current_heading = correct_wrap(current_heading);
  last_euler = current_euler;
    
    
      float error = Gps_heading - *current_heading;
  error=abs(error);
  if(error>180)
  
    current_heading =(current_heading+360);
    current_heading=current_heading+Gps_heading;
    current_heading= current_heading -360;
    current_heading = correct_wrap(current_heading);
  
  else
  %Complement GPS data with regularly updated gyro data
  current_heading = GPS_COEF * Gps_heading + (1 - GPS_COEF) * (*current_heading);
  %Correct 360 deg wrap around
  current_heading = correct_wrap(current_heading);
  end
  
  
  pause();
  end   