Ts_bp = 0.1;
simtime = 0:Ts_bp:25;

% set the following true for the aggressive driver scenario, false for the
% passive drive scenario.
aggressive_driver_use_case = true;

mm1 = ZeroAccelerationAndLateralVelMotionModel(Ts_bp);
mm2 = ConstantAccelerationZeroLateralVelMotionModel(Ts_bp);

if aggressive_driver_use_case == true
    mm3 = LeftLaneChangeRelativeMotionModelWithAcc(Ts_bp, 70, 3.5);
else
    mm3 = LeftLaneChangeRelativeMotionModelWithAcc(Ts_bp, 100, 3.5);

end

u = 0;

% for passive driver
lane_change_initiation_time = 10; 

if aggressive_driver_use_case == true
    lane_change_initiation_time = 15;
end

% Start with constant acceleration
X_aug = [0 10 0 0 0 0]';
% mm = LeftLaneChangeRelativeMotionModel(Ts_bp, 100, 3.5);
% x = [0 10 0 0 0]';
laneChangeMeas = {};
% laneChangeMeas(1).x = X(1);
% laneChangeMeas(1).y = X(4);

% time at which the lane change is complete. This comes out to be approx
% the same for both scenarios and is just a mere coincidence
lc_time = 19.3;

enable_process_noise = false;
for i = 1:length(simtime)
    t = simtime(i);
    
    if aggressive_driver_use_case == true
        if t>= 10 && t < 11        
            % apply 1 m/s2 acceleration as input for 1 second.
            u = 0.5;
            lc_time = 20.2;
        else
            u = 0;
        end
    end
    
    if t == lane_change_initiation_time
        % store the maneuver start point
        x_mid = X_aug(1);
    end
    
    if aggressive_driver_use_case == true
       if t < 10
            X_aug = mm1.propagate(X_aug, 0);
       elseif t >= 10 && t < 15
            X_aug = mm2.propagate(X_aug, u);
       elseif t >=15 && t < lc_time
            X_lc = [X_aug; x_mid];
            X_lc = mm3.propagate(X_lc, u);
            X_aug = X_lc(1:6);
       elseif t >=lc_time
            X_aug = mm2.propagate(X_aug, u);
       end
    else
       if t < 10
            X_aug = mm1.propagate(X_aug, 0);
       elseif t >=lane_change_initiation_time && t < lc_time
            X_lc = [X_aug; x_mid];
            X_lc = mm3.propagate(X_lc, u);
            X_aug = X_lc(1:6);
       elseif t >=lc_time
            X_aug = mm2.propagate(X_aug, u);
       end
    end
                
    laneChangeMeas(i).x = X_aug(1);
    laneChangeMeas(i).y = X_aug(4);
    if enable_process_noise
        X_aug = addProcessNoise(X_aug,Ts_bp);
    end
    laneChangeMeas(i).estimates = X_aug;
end

groundTruth = struct.empty(0,length(simtime));

for i = 1:length(simtime)
    t = simtime(i);
%     [y_gt, X_GT] = generateGroundTruth(X_GT, acc);
%     [y_tilde, X] = generateMeasurement(X,acc);
    groundTruth(i).t = t;
    groundTruth(i).gt_states = laneChangeMeas(i).estimates;
    groundTruth(i).states = laneChangeMeas(i).estimates;
    noisy_meas = generateNoisyMeasurement(laneChangeMeas(i).estimates);
    groundTruth(i).y_tilde = noisy_meas;
    groundTruth(i).y_gt = [laneChangeMeas(i).x; laneChangeMeas(i).y];
end

contextual_IMM_main;

% figure(1);
% plot(simtime, [laneChangeMeas(:).y]);
% 
% figure(2);
% plot([laneChangeMeas(:).x], [laneChangeMeas(:).y]);
% % plot([laneChangeMeas(:).x], [laneChangeMeas(:).y]);
% figure(3);
% plot(simtime, [laneChangeMeas(:).estimates]);


function x = addProcessNoise(X, Ts_bp)
% process noise std dev
%    sigma_matrix = diag([0.16 0.16 0.16 0.16]);
% sigma_matrix = diag([0.032 0.032 0.032 0.032 0.032 0.032]);
sigma_matrix = diag([0.071 0.071 0.071 0.071 0.071 0.071]);
% sigma_matrix = diag([0.1 0.1 0.1 0.1 0.1 0.1]);
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
