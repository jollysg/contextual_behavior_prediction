% MIT License
%
% Copyright (c) 2020 Jasprit Singh Gill (jaspritsgill@gmail.com)
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:

% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.

% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.

Ts_bp = 0.1;
simtime = 0:Ts_bp:30;

% set the following true for the aggressive driver scenario, false for the
% passive drive scenario.
aggressive_driver_use_case = false;

mm1 = ZeroAccelerationAndLateralVelMotionModel(Ts_bp);
mm2 = ConstantAccelerationZeroLateralVelMotionModel(Ts_bp);

if aggressive_driver_use_case == false
    mm3 = LeftLaneChangeRelativeMotionModelWithAcc(Ts_bp, 100, 3.5);
else
    mm3 = LeftLaneChangeRelativeMotionModelWithAcc(Ts_bp, 70, 3.5);

end

% No acceleration input by default
u = 0;

% for passive driver
lane_change_initiation_time = 10; 

if aggressive_driver_use_case == true
    lane_change_initiation_time = 15;
end

% Start with constant acceleration
X_aug = [0 10 0 0 0 0]';
laneChangeMeas = {};

% time at which the lane change is complete. This comes out to be approx
% the same for both scenarios and is just a mere coincidence
lc_time = 19.3;

enable_process_noise = false;
enable_measurement_noise = true;

for i = 1:length(simtime)
    t = simtime(i);
    
    if aggressive_driver_use_case == true
        if t>= 10 && t < 11        
            % apply 1 m/s2 acceleration as input for 1 second.
            u = 0.5;
            lc_time = 20.2;
%             u = 1;
%             lc_time = 19.3;
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
    groundTruth(i).t = t;
    groundTruth(i).gt_states = laneChangeMeas(i).estimates;
    groundTruth(i).states = laneChangeMeas(i).estimates;
    noisy_meas = generateMeasurement(laneChangeMeas(i).estimates, enable_measurement_noise);
    groundTruth(i).y_tilde = noisy_meas;
    groundTruth(i).y_gt = [laneChangeMeas(i).x; laneChangeMeas(i).y];
end

simtime = 0:Ts_bp:25;

contextual_IMM_main;

% debug visualizations for y axis of lane change vs time
% figure(1);
% plot(simtime, [laneChangeMeas(:).y]);
% 

% debug visualizations for y axis of lane change vs x.
% figure(2);
% plot([laneChangeMeas(:).x], [laneChangeMeas(:).y]);

% figure(3);
% plot(simtime, [laneChangeMeas(:).estimates]);


function x = addProcessNoise(X, Ts_bp)
    % process noise std dev
    %    sigma_matrix = diag([0.16 0.16 0.16 0.16]);
    % sigma_matrix = diag([0.032 0.032 0.032 0.032 0.032 0.032]);
    % sigma_matrix = diag([0.1 0.1 0.1 0.1 0.1 0.1]);
    %     sigma_matrix = diag([0 0 0 0 0 0]);
    sigma_matrix = diag([0.071 0.071 0.071 0.071 0.071 0.071]);
    x = X + sqrt(Ts_bp)*sigma_matrix*randn(6,1);
end

function y = generateMeasurement(X, enable_measurement_noise)
    Ck = [1 0 0 0 0 0; 0 0 0 1 0 0];

    % measurement noise std_dev
    %     R_sigma = [0.5 0; 0 0.5];
        
    if enable_measurement_noise == true
        R_sigma = [0.05 0; 0 0.05];
    else
        R_sigma = [0.0 0; 0 0.0];
    end
    % Measurement noise is being added externally using the random number block
    y = Ck * X + R_sigma * randn(2,1);
end
