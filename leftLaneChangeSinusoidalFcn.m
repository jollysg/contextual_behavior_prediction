function x_apriori = leftLaneChangeSinusoidalFcn(x,u)
%LEFTLANECHANGESINUSOIDALFCN Summary of this function goes here
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
x_apriori = [x(1) + x(2)*Ts_bp; x(2); x(3) + x(4)*Ts_bp; -A*w*x(2)*sin(w*x(1)-pi)] + u;
end

