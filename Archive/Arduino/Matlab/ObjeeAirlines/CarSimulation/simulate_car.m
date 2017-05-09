% Car Simluation
% Kent Altobelli
% 30 Oct 2015

function [new_lat, new_long, new_gyro_head, new_GPS_head, new_true_head] = simulate_car(old_lat, old_long, old_gyro_head, throttle, steering, last_true_head, dt)
% Car simulation given throttle and steering inputs
% Generates feedback "sensor data" to be used by control system

% Set car characteristic constants
throttle_range = [0 100];  % throttle input value
max_speed = 5;  % knots at full throttle
steering_range = [-100 100];
max_turn_rate = 90;  % degrees per second at max throttle, decrease linearly

% Conversion constants
ms_per_knot = .514;  % 1 knot = .514 m/s
m_per_degree = 111320;  % 111320 meters per deg of latitude


% Find speed and turn rate from inputs
speed = (max_speed / throttle_range(2)) * throttle;
turn_rate =  (max_turn_rate / steering_range(2)) * steering;
turn_deg = turn_rate * dt;

% Find car's new location given throttle and steering
new_true_head = last_true_head



end

