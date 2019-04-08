% initial conditions
x0 = 0;      % x position (m)
y0 = 0;      % y position (m)
vx0 = 10;    % longitudinal velocity (m/s)
vy0 = 0;     % lateral velocity (m/s)
theta0 = 0; % heading / yaw (radians)
w0 = 0.157;  % yaw rate / turn rate (rad/sec)
ax0 = 1;    % longitudinal acceleration (m/sec2)
R = -vx0/w0;   % radius of road curvature (m)
c = 1/R;    % road curvature (1/m)

stime = .1;   % sample time (secs)
simtime = 0:stime:5;    %simulation time (secs)
n = length(simtime);

% constant velocity model
X0 = [x0 vx0 y0 vy0]';
CV_Model = zeros(length(X0), n);
CV_Model(:,1) = X0;

X0 = [x0 y0 theta0 vx0 w0]';
CTRV_Model = zeros(length(X0), n);
CTRV_Model(:,1) = X0;

X0 = [x0 y0 theta0 vx0 ax0 w0]';
CTRA_Model = zeros(length(X0), n);
CTRA_Model(:,1) = X0;

X0 = [x0 y0 theta0 vx0 ax0 c]';
CCA_Model = zeros(length(X0), n);
CCA_Model(:,1) = X0;

%XCTRV0 = [x0 y0 theta0 v0 w0]';

for i = 1 : n-1
    CV_Model(:, i+1) = cv_model_fun(CV_Model(:, i), 0, stime);
    CTRV_Model(:, i+1) = ctrv_model_fun(CTRV_Model(:, i), 0, stime);
    CTRA_Model(:, i+1) = ctra_model_fun(CTRA_Model(:,i), 0, stime);
    CCA_Model(:, i+1) = cca_model_fun(CCA_Model(:,i), 0, stime);
%     plot(x,y);
%     hold on
end

plot(CV_Model(1,:), CV_Model(3, :), 'o', ...
    CTRV_Model(1,:), CTRV_Model(2,:), 'x', ...
    CTRA_Model(1,:), CTRA_Model(2,:), '+', ...
    CCA_Model(1,:), CCA_Model(2,:), '*')
legend('CV Model', 'CTRV Model', 'CTRA Model', 'CCA Model');
xlabel('X position (m)');
ylabel('Y position (m)');
grid on;
% 
% function x_plus_1 = cv_model_fun(X, Y, dt, w)
%     % X = [x vx y vy]'
%     x = X(1);
%     vx = X(2);
%     y = X(3);
%     vy = X(4);    
%     x_plus_1 = [x + dt * vx; vx; y + dt * vy; vy];
% end
% 
% function x_plus_1 = ctrv_model_fun(X, Y, dt, w)
%     % X = [x y theta v w]'
%     x = X(1);
%     y = X(2);
%     theta = X(3);
%     v = X(4);
%     w = X(5);
%     x_plus = v/w*sin(w * dt + theta) - v/w * sin(theta) + x;
% %     y_plus = -v/w * cos(w * dt + theta) + v/w * sin(theta) + y;
%     y_plus = -v/w * cos(w * dt + theta) + v/w * cos(theta) + y;
%     x_plus_1 = [x_plus; y_plus; w*dt + theta; v; w];
% end
% 
% function x_plus_1 = ctra_model_fun(X, Y, dt, w)
%     % X = [x y theta v a w]'
%     x = X(1);
%     y = X(2);
%     theta = X(3);
%     v = X(4);
%     a = X(5);
%     w = X(6);
%     x_plus = 1/w^2*((v*w + a*w*dt) * sin(theta + w*dt) + a*cos(theta + w*dt) - v*w*sin(theta) - a*cos(theta)) + x;
%     y_plus = 1/w^2*((-v*w - a*w*dt) * cos(theta + w*dt) + a*sin(theta +w*dt) + v*w*cos(theta) - a*sin(theta)) + y;
%     x_plus_1 = [x_plus; y_plus; w*dt + theta; a*dt + v; a; w];
% end
% 
% function x_plus_1 = cca_model_fun(X, Y, dt, w)
%     % X = [x y theta v a c]'
%     x = X(1);
%     y = X(2);
%     theta = X(3);
%     v = X(4);
%     a = X(5);
%     c = X(6);
%     gamma1 = 1/4/a*(c*v^2 + 4*a*theta);
%     gamma2 = c*dt*v + c*dt^2*a - theta;
%     neta = sqrt(2*pi)*v*c;  % TODO: check this if this is sqrt(2*pi*v*c) in the paper
%     zeta1 = (2*a*dt + v) * sqrt(c/(2*a*pi));
%     zeta2 = v * sqrt(c / (2*a*pi));
%     
%     
%     x_plus = 1/ (4 * sqrt(a*c) * c ) * ...
%         (neta * cos(gamma1)*fresnelc(zeta1) ...
%         + neta * sin(gamma1)*fresnels(zeta1) ...
%         - neta * cos(gamma1)*fresnelc(zeta2) ...
%         - neta * sin(gamma1)*fresnels(zeta2) ...
%         + 2 * sin(gamma2) * sqrt(a*c) + 2 * sin(theta)*sqrt(a*c))...
%         + x;
%     y_plus = 1/ (4 * sqrt(a*c) * c ) * ...
%         (- neta * cos(gamma1)*fresnels(zeta1) ...
%         + neta * sin(gamma1)*fresnelc(zeta1) ...
%         - neta * sin(gamma1)*fresnelc(zeta2) ...
%         + neta * cos(gamma1)*fresnels(zeta2) ...
%         + 2 * cos(gamma2) * sqrt(a*c) - 2 * cos(theta)*sqrt(a*c))...
%         + y;
%     x_plus_1 = [x_plus; y_plus; -1/2*c*dt^2 *a - c*dt*v + theta; a*dt + v; a; c];
% end
% 
