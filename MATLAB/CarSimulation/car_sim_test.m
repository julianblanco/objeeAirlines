% Simple test for car simulator

clf;
clc;
clear all;

old_gyro_head = 0;
old_true_head = 360*rand;
old_lat = 41.3723;
old_long = -72.1019;

throttle = 50;
dt = .1;

% TODO get linear and circular approximations to mesh
%printf('car_sim_test.m - WORK IN PROGRESS\n');

<<<<<<< HEAD
for (t = 0:dt:10)
=======
for t = 0:dt:5
>>>>>>> bca5b19db8bad5ce1f2129b0c92851c5ff543281
steering = 100 * sin(2*t) + eps;

[new_lat, new_long, new_gyro_head, new_GPS_head, new_true_head] = simulate_car(old_lat, old_long, old_gyro_head, throttle, steering, old_true_head, dt);

% Set values for next loop
old_lat = new_lat;
old_long = new_long;
old_true_head = new_true_head;
old_gryo_head = new_gyro_head;


% Debug info
%printf('true = %.1f, gyro = %.1f\n', new_true_head, new_gyro_head);

<<<<<<< HEAD
pause(dt);
endfor
=======
pause(dt/100);
end
>>>>>>> bca5b19db8bad5ce1f2129b0c92851c5ff543281
