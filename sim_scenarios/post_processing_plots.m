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

figure(1)
plot(simtime, [filter_traj(:).driver_weights], 'Linewidth', 2);
legend('Aggr. driver', 'Passive driver');
legend('Location', 'northwest');
xlabel('Time (seconds)');
ylabel('Driver weights (probabilities)');
title('Driver weights vs time');
ylim([-0.1 1.19]);
grid on;

figure(2)
plot(simtime, [filter_traj(:).likelihoods]);
legend('Straight pass', 'straight agg', 'left lane pass', 'left lane agg');
xlabel('Time (seconds)');
ylabel('likelihood of filters');
title('Filter likelihoods vs time');
lim = ylim;
lim(1) = -5;
ylim(lim)
grid on;

figure(3)
plot(simtime, [filter_traj(:).weights], 'Linewidth', 1.5);
legend('Straight pass', 'straight agg', 'left lane pass', 'left lane agg');
legend('Location', 'northwest');
xlabel('Time (seconds)');
ylabel('Filter probabilities');
title('Filter probabilistic weights vs time');
ylim([-0.1 1.19]);
grid on;

estimate_flt1 = [];
for i = 1:length(filter_traj)
    estimate_flt1(:,i) = squeeze(filter_traj(i).estimates(:,:,1));
end

figure(4)
plot(simtime, estimate_flt1);
legend('x', 'vx', 'ax', 'y', 'vy', 'ay');
legend('Location', 'northwest');
xlabel('Time (seconds)');
ylabel('State estimate Filter 1');
title('State estimates vs time');
grid on;

figure(5)
plot(simtime, [filter_traj(:).combined_estimates]);
legend('x', 'vx', 'ax', 'y', 'vy', 'ay');
xlabel('Time (seconds)');
ylabel('Combined estimates');
legend('Location', 'northwest');
title('Combined estimates vs time');
grid on;

figure(6)
combined_estim_traj = [filter_traj(:).combined_estimates];
y_gt = [groundTruth(:).y_gt];
meas = [groundTruth(:).y_tilde];
plot(y_gt(1,:), y_gt(2,:), 'r', ...
    meas(1,:), meas(2,:), 'g', ...
    combined_estim_traj(1,:), combined_estim_traj(4,:), 'b', ...
    'Linewidth', 1.5);
title('Estimates vs ground truth of position trajectories');
xlabel('x position (meters)');
ylabel('y position (meters)');
legend('ground truth', 'meas', 'estimates');
legend('Location', 'northwest');
ylim([-1.75 3.5+1.75]);

estim_comb = [combined_estim_traj(1,:); combined_estim_traj(4,:)];

% Requires signal processing toolbox
% rms_error = rms(estim_comb' - y_gt')
