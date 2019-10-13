function dfdx = leftLaneChangeParametricJacobianFcn(x,u,len,lateral_dist)
%LEFTLANECHANGESINUSOIDALFCN Summary of this function goes here
% Sinusoidal lane change function that accepts the length and amplitude as
% parameters. length is maneuver length, lateral_dist is amplitude of the
% maneuver
%   Detailed explanation goes here
%tim = len / x(2);
% y_init = lateral_dist;         % signifies the starting point in y
% x_init = 0;

A = lateral_dist/2;
f = 1/(2*len);
w = 2*pi*f;
Ts = 0.1;
dfdx = [1 Ts 0 0; 0 1 0 0; 0 0 1 Ts; A*w^2*x(2)*cos(w*x(1)-pi) -A*w*sin(w*x(1)-pi) 0 0];
end

