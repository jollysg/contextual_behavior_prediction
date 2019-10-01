function dfdx = leftLaneChangeRelativeJacobianFcn(x,u,len,lateral_dist)
%LEFTLANECHANGESINUSOIDALFCN Summary of this function goes here
% Sinusoidal lane change function that accepts the length and amplitude as
% parameters. length is maneuver length, lateral_dist is amplitude of the
% maneuver. Format of states X = [x, vx, y, vy, rel_x], where, x is
% position, y is position, vx and vy are resp. velocities and rel_x is the
% relative longitudinal position from the start of the maneuver. Adding
% rel_x as a state. 
%   Detailed explanation goes here
%tim = len / x(2);
% y_init = lateral_dist;         % signifies the starting point in y
% x_init = 0;

A = lateral_dist/2;
f = 1/(2*len);
w = 2*pi*f;
Ts = 0.01;
dfdx = [1 Ts 0 0 0; 0 1 0 0 0; A*w*sin(w*(x(1)-x(5))) 0 0 0 -A*w*sin(w*(x(1)-x(5))); A*w*w*x(2)*cos(w*(x(1)-x(5))) A*w*sin(w*(x(1)-x(5))) 0 0 -A*w^2*x(2)*cos(w*(x(1)-x(5))); 0 0 0 0 1];
% dfdx = [1 Ts 0 0 0; 0 1 0 0 0; 0 0 0 0 A*w*sin(w*x(5)); 0 A*w*sin(w*x(5)) 0 0 A*w^2*x(2)*sin(w*x(5)); 0 Ts 0 0 1];
%dfdx = [1 Ts 0 0 0; 0 1 0 0 0; 0 0 0 0 A*w*sin(w*x(5)); 0 A*w*sin(w*x(5)) 0 0 A*w^2*x(2)*sin(w*x(5)); 0 0 0 0 1];
end

