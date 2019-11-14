function dfdx = rightLaneChangeParametricJacobianFcn(x,u,len,lateral_dist)
%LANECHANGESINUSOIDALJA Summary of this function goes here
%   Detailed explanation goes here
A = lateral_dist/2;
f = 1/(2*len);
w = 2*pi*f;
Ts_bp = 0.1;
dfdx = [1 Ts_bp 0 0; 0 1 0 0; 0 0 1 Ts_bp; -A*w^2*x(2)*cos(w*x(1)) -A*w*sin(w*x(1)) 0 0];
end
