y_trajectory = vehicle1.trajectories_y;
times = vehicle1.times_s;

% This is a diff between the adjacent elements of y_trajectory and is
% velocity in m/0.1s
vy_trajectory_mpms = [0;diff(y_trajectory)];


% Since the time interval for the trajectories is 0.1s, (100 ms), the
% vecolity in m/s is vy_trjaecotry * 10 in m/s
vy_trajectory = vy_trajectory_mpms*10;


%vy now has velocity in m/s
findStatsFromVector(vy_trajectory_mpms);

figure(1);
plot(times, vy_trajectory_mpms);

%calculating the acceleartion in m / (0.1s)2:
ay_trajectory_mpms = [0; diff(vy_trajectory_mpms)];

figure(2);
plot(times, ay_trajectory_mpms);

