clear;

x = 0;
vx = 10;
y = 0;
vy = 0;
ay = 0;
rel_x = 0;

X = [x vx y vy]';

%Ts_bp = 0.01;
Ts_bp = 0.1;

% covariance of the initial state conditions, i.e. priors
P_init = 1;


%time for the maneuver
maneuver_length = 150;   % in meters
tim = maneuver_length / vx;
lanewidth = 3.5;
f = 1/(2*maneuver_length);
w = 2*pi*f;
A = lanewidth/2 ;   % from middle of the lane to the middle
y_init = lanewidth/2;         % signifies the starting point in y
x_init = 0;


%
% vx = 3.81;
% x = 14.6953;
% y = 23.401;
ax = 0;
