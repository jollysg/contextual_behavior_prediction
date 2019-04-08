x = 0;
vx = 10;
y = 0;
vy = 0;

X = [x vx y vy]';

Ts_bp = 0.01;


%time for the maneuver
maneuver_length = 100;   % in meters
tim = maneuver_length / vx;
lanewidth = 3.5;
f = 1/(2*maneuver_length);
w = 2*pi*f;
A = lanewidth/2 ;   % from middle of the lane to the middle
y_init = lanewidth/2;         % signifies the starting point in y
x_init = 0;