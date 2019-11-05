Ts_bp = 0.1;
simtime = 0:Ts_bp:25;
mm1 = ZeroAccelerationAndLateralVelMotionModel(Ts_bp);
mm2 = ConstantAccelerationZeroLateralVelMotionModel(Ts_bp);
mm3 = LeftLaneChangeRelativeMotionModelWithAcc(Ts_bp, 70, 3.5);

% Start with constant acceleration
X_aug = [0 10 0 0 0 0]';
% mm = LeftLaneChangeRelativeMotionModel(Ts_bp, 100, 3.5);
% x = [0 10 0 0 0]';
laneChangeMeas = {};
% laneChangeMeas(1).x = X(1);
% laneChangeMeas(1).y = X(4);
for i = 1:length(simtime)
    t = simtime(i);
    if t>= 10 && t < 11
        % apply 1 m/s2 acceleration as input for 1 second.
        u = 1;
    else
        u = 0;
    end
    
    if t == 15
        % store the maneuver start point
        x_mid = X_aug(1);
    end
    
    if t < 10
        X_aug = mm1.propagate(X_aug, 0);
    elseif t >= 10 && t < 15
        X_aug = mm2.propagate(X_aug, u);
    elseif t >=15 && t < 19.3
        X_lc = [X_aug; x_mid];
        X_lc = mm3.propagate(X_lc, u);
        X_aug = X_lc(1:6);
    elseif t >=19.3
        X_aug = mm2.propagate(X_aug, u);
    end
            
    laneChangeMeas(i).x = X_aug(1);
    laneChangeMeas(i).y = X_aug(4);
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
    groundTruth(i).y_tilde = [laneChangeMeas(i).x; laneChangeMeas(i).y];
    groundTruth(i).y_gt = [laneChangeMeas(i).x; laneChangeMeas(i).y];
end

% contextual_IMM_main;

figure(1);
plot(simtime, [laneChangeMeas(:).y]);

figure(2);
plot([laneChangeMeas(:).x], [laneChangeMeas(:).y]);
% plot([laneChangeMeas(:).x], [laneChangeMeas(:).y]);
figure(3);
plot(simtime, [laneChangeMeas(:).estimates]);
