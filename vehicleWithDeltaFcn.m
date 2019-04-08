function x_apriori = vehicleWithDeltaFcn(x,u)
%LEFTLANECHANGESINUSOIDALFCN Summary of this function goes here
% Sinusoidal lane change function that accepts the length and amplitude as
% parameters. length is maneuver length, lateral_dist is amplitude of the
% maneuver
%   Detailed explanation goes here
%tim = len / x(2);
% y_init = lateral_dist;         % signifies the starting point in y
% x_init = 0;

% A = lateral_dist/2;
f = 1/(2*5);
w = 2*pi*f;
Ts_bp = 0.01;
acc = - A* (w*x(2))^2*cos(w*x(2)-pi);
% x_apriori = [x(1) + x(2)*Ts_bp; x(2); x(3) + x(4)*Ts_bp; -A*w*x(2)*sin(w*x(1)-pi) - A* (w*x(2))^2*cos(w*x(2)-pi)] + u;
x_apriori = [x(1) + x(2)*Ts_bp; x(2); x(3) + x(4)*Ts_bp; -A*w*x(2)*sin(w*x(1)-pi)] + u;

end

