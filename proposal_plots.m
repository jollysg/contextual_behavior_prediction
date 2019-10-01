% subplot(2,1,1);
% plot (ground_truth.Time, weights.Data)
% %plot (ground_truth.Time, weights.Data(:,1), 'k--', ground_truth.Time, weights.Data(:,2), 'r-', ground_truth.Time, weights.Data(:,3), 'b-.')
% title('Left lane change weights against time') 
% legend('50 m length', '100 m length', '150 m length')
% ylim([-0.2 1.2])
fi% ylabel('probability')
% xlabel('Time (secs)')
% grid on;
% 
% subplot(2,1,2);
% plot (ground_truth.Data(:,1), ground_truth.Data(:,2));
% title('Trajectory for left lane change (y position against time)')
% ylabel('Y position (m)');
% xlabel('X position (m)');
% grid on;
figure(1);
subplot(2,1,1);
%plot (ground_truth.Time, weights.Data)
plot (ground_truth.Time, weights.Data(:,1), 'k--', ground_truth.Time, weights.Data(:,2), 'r-', ground_truth.Time, weights.Data(:,3), 'b-.')
title('right lane change weights against time') 
legend('Straight', 'Right lane change', 'Left lane change')
ylim([-0.2 1.2])
ylabel('probability')
xlabel('Time (secs)')
grid on;

subplot(2,1,2);
plot (measurements.Data(:,1), measurements.Data(:,2));
title('measurements for right lane change (y position against time)')
ylabel('Y position (m)');
xlabel('X position (m)');
xlim([0 60])
grid on;

figure(2);
plot (ground_truth.Time, weights.Data(:,1), 'k--', ground_truth.Time, weights.Data(:,2), 'r-', ground_truth.Time, weights.Data(:,3), 'b-.')
title('right lane change weights against time') 
legend('Straight', 'Right lane change', 'Left lane change')
ylim([-0.2 1.2])
ylabel('probability')
xlabel('Time (secs)')
grid on;
