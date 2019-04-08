function dfdx = rightLaneChangeSinusoidalManeuverLenghtJacobianFcn(x, u)
%LANECHANGESINUSOIDALJA Summary of this function goes here
%   Detailed explanation goes here
% maneuver_length = 150;   % in meters
% tim = maneuver_length / x(2);
% f = 1/(2*maneuver_length);
% w = 2*pi*f;
% y_init = lanewidth/2;         % signifies the starting point in y
% x_init = 0;

lanewidth = 3.5;
A = lanewidth/2 ;   % from middle of the lane to the middle
Ts = 0.01;
df5dL = A * pi * x(2)/x(5)^2 * ( sin (pi*x(1)/x(5)) + pi*x(1)/x(5) * cos(pi*x(1)/x(5)) );
dfdx = [1 Ts 0 0 0; 0 1 0 0 0; 0 0 1 Ts 0; -A*(pi/x(5))^2*x(2)*cos(pi/x(5)*x(1)) -A*pi/x(5)*sin(pi/x(5)*x(1)) 0 0 df5dL; 0 0 0 0 1];
end

