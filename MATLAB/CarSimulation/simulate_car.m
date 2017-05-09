% Car Simluation
% Kent Altobelli
% 30 Oct 2015

function [new_lat, new_long, new_gyro_head, new_GPS_head, new_true_head] = simulate_car(old_lat, old_long, old_gyro_head, throttle, steering, old_true_head, dt)
% Car simulation given throttle and steering inputs
% Generates feedback "sensor data" to be used by control system

% Set car characteristic constants
throttle_range = [0 100];  % throttle input value
max_speed = 5;  % knots at full throttle
steering_range = [-100 100];  % range of steering input value **positive is right**
max_turn_rate = 90;  % degrees per second at max throttle, decrease linearly

% Conversion constants
ms_per_knot = .514;  % 1 knot = .514 m/s
m_per_degree = 111120;  % 111120 meters per deg of latitude


% Find speed -> distance driven, turn rate -> degrees of turn from incoming data
speed = (max_speed / throttle_range(2)) * throttle * ms_per_knot;  % m/s
dist_travel = speed * dt;
turn_rate =  (max_turn_rate / steering_range(2)) * steering;  % deg/s
turn_deg = turn_rate * dt;

% Use linear path approximation to find next step length and direction
%if (abs(turn_deg) < 50)
  result_mag = dist_travel;
  result_ang = abs(turn_deg);
%else
%  % Find circular path model to estimate parallel and perpendicular paths to orginal course
%  radius = dist_travel / deg2rad(abs(turn_deg));
%  advance = radius * cos(deg2rad(abs(turn_deg)));
%  transfer = radius * (1 - sin(deg2rad(abs(turn_deg))));
%  result_mag = sqrt(advance^2 + transfer^2);
%  result_ang = rad2deg(atan(transfer / advance));
%endif

% Use parallel and perpendicular estimates to generate new lat and long positions
new_GPS_head = correct_wrap(old_true_head + result_ang*(steering/abs(steering)));
new_long = old_long + (result_mag/m_per_degree)*cos(deg2rad(nav2math_angle(new_GPS_head)));
new_lat = old_lat + (result_mag/m_per_degree)*sin(deg2rad(nav2math_angle(new_GPS_head)));


% Update true heading and gyro heading
new_true_head = correct_wrap(old_true_head + turn_deg);
new_gyro_head = correct_wrap(old_gyro_head + turn_deg);


% Plot new values
figure(1);
hold on;
grid on;
plot(new_long, new_lat);

% Debug info
%printf('turn_deg = %.2f, radius = %.1f, transfer = %.3f\n', turn_deg, radius, transfer);

endfunction