% post processing after running mmae_imm_rel.slx simulink model

weights = out.imm_weights.Data(1,:,:);
weights = squeeze(weights);

time_series = out.tout;

est = [out.imm_comb_est.Data(:,1) out.imm_comb_est.Data(:,3)];

meas = [out.meas.Data(:,1) out.meas.Data(:,2)];
gt.x = out.groundTruth_imm.Signal_1.Data;
gt.y = out.groundTruth_imm.Signal_2.Data;

tiledlayout(2,1);

nexttile;
plot(gt.x, gt.y, est(:,1), est(:,2), meas(:,1), meas(:,2), 'Linewidth', 1.5);
legend('Ground truth', 'Estimate', 'Measurement');
legend('Location', 'northwest');
xlabel('Longitudinal position (m)');
ylabel('Lateral position (m)');
title('Trajectory of the observed traffic participant');
ylim ([-1 4.5]);

nexttile;
plot(time_series, weights, 'Linewidth', 1.5);
title('Probabilistic weights of maneuvers vs time');
ylabel('Time (seconds)');
xlabel('Probabilistic weights');
ylim ([-0.1 1.5]);
% legend('Straight maneuver', 'Left lane change', 'Right lane change');
legend('Straight maneuver', 'Left lane change short', 'Left lane change long',...
        'Right lane change short', 'Right lane change long');

rms(meas-est)
% 0.0798    0.0368
