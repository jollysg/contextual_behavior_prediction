% AMM custom signal, with resetting logic, plotting file

figure(1);
tiledlayout(3,1);
nexttile
plot(gt_custom.Signal_1.Data(:), gt_custom.Signal_2.Data(:), ...
    comb_estimate.Data(:,1), comb_estimate.Data(:,3));
ylim([-1 4.5]);
title('Trajectory of the vehicle');
xlabel('X position (m)');
ylabel('Y position (m)');
legend('Ground truth', 'Combined estimate');

nexttile
%straight_likelihood.Time, straight_likelihood.Data(:), ...
plot(moving_avgs.Time, moving_avgs.Data(:,1), ...
            moving_avgs.Time, moving_avgs.Data(:,2));
xlabel('Time (seconds)');
ylabel('likelihood of default maneuver (straight)');
title('Smoothing the likelihood of the default maneuver');
legend('Moving average N = 5', 'Cascaded moving avg N1=5, N2=10');


nexttile
plot(reset_trigger.Time, reset_trigger.Data(:)/4, ...
            wrr.Time, wrr.Data(:));
%     increasing_likelihood.Time, increasing_likelihood.Data(:), ...
%     decreasing_likelihood.Time, decreasing_likelihood.Data(:), ...
legend('Reset pulse based on likelihood thresholds', 'Weighted recency averaging divergence');
xlabel('Time (seconds)');
ylabel('Magnitude');
ylim([-20 20]);

figure(2);
plot(left_lane_est.Time, left_lane_est.Data(:,:));
legend('x position (m)', 'long. velocity (m/s)', 'y position (m)', 'lat. velocity (m/s)', 'x_{mip}(m)');
legend('Location', 'northwest');
xlabel('Time (seconds)');

figure(3);
plot(weights.Time, weights.Data(:,:));
ylim([-0.1 1.4]);
xlabel('Time (seconds)');
ylabel('Probabilistic weights');
legend('Straight maneuver', 'Right lane change', 'Left Lane change');
title('Probabilistic weights of the AMM elemental filters');