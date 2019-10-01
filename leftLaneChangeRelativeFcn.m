function x_apriori = leftLaneChangeRelativeFcn(x,u,len,lateral_dist)
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
Ts_bp = 0.01;
 x_apriori = [x(1) + x(2)*Ts_bp; x(2); -A * cos(w*(x(1)-x(5))) + A; A*w*x(2)*sin(w*(x(1)-x(5))); x(5)] + u;
% x_apriori = [x(1) + x(2)*Ts_bp; x(2); -A * cos(w*x(5)) + A; A*w*x(2)*sin(w*x(5)); x(5)] + u;
end

