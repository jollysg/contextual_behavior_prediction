function dfdx = laneChangeSinusoidalJacobianFcn(x, u)
%LANECHANGESINUSOIDALJA Summary of this function goes here
%   Detailed explanation goes here
maneuver_length = 150;   % in meters
tim = maneuver_length / x(2);
lanewidth = 3.5;
f = 1/(2*maneuver_length);
w = 2*pi*f;
A = lanewidth/2 ;   % from middle of the lane to the middle
y_init = lanewidth/2;         % signifies the starting point in y
x_init = 0;
Ts_bp = 0.01;
dfdx = [1 Ts_bp 0 0; 0 1 0 0; 0 0 1 Ts_bp; -A*w^2*x(2)*cos(w*x(1)) -A*w*sin(w*x(1)) 0 0];
end

