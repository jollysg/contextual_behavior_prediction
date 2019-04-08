function x_apriori = rightLaneChangeSinusoidalManeuverLengthFcn(x,u)
%LANECHANGESINUSOIDAL Summary of this function goes here
%   Detailed explanation goes here
%   arguments = x - [x vx y vy L]
% maneuver_length = 150;   % in meters
% tim = maneuver_length / x(2);
% f = 1/(2*maneuver_length);
% w = 2*pi*f;
% y_init = lanewidth/2;         % signifies the starting point in y
% x_init = 0;

lanewidth = 3.5;
A = lanewidth/2 ;   % from middle of the lane to the middle
Ts = 0.01;
x_apriori = [x(1) + x(2)*Ts; x(2); x(3) + x(4)*Ts; -A*pi/x(5)*x(2)*sin(pi/x(5)*x(1)); x(5)] + u;
end
