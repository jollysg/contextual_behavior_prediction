% chap 6 plots

figure(1);
straight_wts = prenorm_str.Data;
left_wts = prenorm_left.Data;
right_wts = prenorm_right.Data;
time_ser = prenorm_left.Time;
plot(time_ser, straight_wts, time_ser, left_wts, time_ser, right_wts, 'Linewidth', 1.5);
legend('straight', 'left lane change', 'right lane change');
xlabel('Time (seconds)');
ylabel('Weighted likelihood');
ylim([0, 60]);
title('Prenormalized weights for maneuvers against time');


figure(2);
straight_wts = likelihood_str.Data;
left_wts = likelihood_left.Data;
right_wts = likelihood_rt.Data;
% time_ser = prenorm_left.Time;
plot(time_ser, straight_wts, time_ser, left_wts, time_ser, right_wts, 'Linewidth', 1.5);
legend('straight', 'left lane change', 'right lane change');
xlabel('Time (seconds)');
ylabel('Likelihood');
ylim([0, 60]);
title('Likelihood for maneuvers against time');