function y = positionMeasurementFcn(x,u)
%POSITIONMEASUREMENTFCN For constant velocity model
%   Detailed explanation goes here
%   Arg1 x = [x vx y vy]
%   Arg2 u = 0

y = [x(1) + u(1); x(3) + u(3)];
end

