
Ts_bp = 0.1;
simtime = 0:Ts_bp:25;

groundTruth = struct.empty(0,length(simtime));
X_init = [x vx ax y vy ay]';
no_of_states = length(X_init);
X = X_init;
X_GT = X_init;
filter_traj = struct.empty(0,length(simtime));
acc = 0;

for i = 1:length(simtime)
    t = simtime(i);
    %Following code adds acceleration of 1m/s after 10 seconds for 1
    %second. As a result, the vehicle accelerates by 1 m/s after this.
    %Comment the following IF condition for a constant velocity run.
%     if t >= 10 && t < 11
%         acc = 1;
%     else
%         acc = 0;
%     end
    [y_gt, X_GT] = generateGroundTruth(X_GT, acc);
    [y_tilde, X] = generateMeasurement(X,acc);
    groundTruth(i).t = t;
    groundTruth(i).gt_states = X_GT;
    groundTruth(i).states = X;
    groundTruth(i).y_tilde = y_tilde;
    groundTruth(i).y_gt = y_gt;
end

contextual_IMM_main;


function [y, perturbed_states] = generateMeasurement(X, u)
Ts_bp = 0.1;
% states - X = [x vx y vy] -
Ak = [  1   Ts_bp   Ts_bp^2/2   0   0   0;
    0   1       Ts_bp       0   0   0;
    0   0       1           0   0   0;
    0   0       0           1 Ts_bp Ts_bp^2/2;
    0   0       0           0   1   Ts_bp ;
    0   0       0           0   0   1];
Bk = [0; 0; 1; 0; 0; 0] * Ts_bp;

%     X(5) = 0;
X(4) = 0;

X_GT = Ak * X + Bk * u;
perturbed_states = addProcessNoise(X_GT, Ts_bp);

y = generateNoisyMeasurement(X_GT);
end

function x = addProcessNoise(X, Ts_bp)
% process noise std dev
%    sigma_matrix = diag([0.16 0.16 0.16 0.16]);
sigma_matrix = diag([0.032 0.032 0.032 0.032 0.032 0.032]);
%     sigma_matrix = diag([0 0 0 0 0 0]);
x = X + sqrt(Ts_bp)*sigma_matrix*randn(6,1);
end

function y = generateNoisyMeasurement(X)
Ck = [1 0 0 0 0 0; 0 0 0 1 0 0];

% measurement noise std_dev
%     R_sigma = [0.5 0; 0 0.5];
R_sigma = [0.05 0; 0 0.05];

% Measurement noise is being added externally using the random number block
% R_sigma = [0.0 0; 0 0.0];
y = Ck * X + R_sigma * randn(2,1);
end

function y = generateNoiselessMeasurement(X)
Ck = [1 0 0 0 0 0; 0 0 0 1 0 0];
y = Ck * X;
end
function [y, X_GT] = generateGroundTruth(X, u)
Ts_bp = 0.1;
% states - X = [x vx y vy] -
Ak = [  1   Ts_bp   Ts_bp^2/2   0   0   0;
    0   1       Ts_bp       0   0   0;
    0   0       1           0   0   0;
    0   0       0           1 Ts_bp Ts_bp^2/2;
    0   0       0           0   1   Ts_bp ;
    0   0       0           0   0   1];
Bk = [0; 0; 1; 0; 0; 0] * Ts_bp;
Ck = [1 0 0 0 0 0; 0 0 0 1 0 0];

%     X(5) = 0;
X(4) = 0;

X_GT = Ak * X + Bk * u;
y = generateNoiselessMeasurement(X_GT);
end