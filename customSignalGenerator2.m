% load('/home/jasprit/Documents/research/Behavior_identification_for_maneuver/ground_truth_LC.mat');
lane_change_measurements_generator;
% generates parts of the maneuver, straight, lane change and then straight
%straight - 10 seconds
firstSetTimeSeries = 0:0.1:9.90;
%lane change - 5 seconds
secondSetTimeSeries = simtime + 10;
%straight - 5 seconds
st_time = secondSetTimeSeries(end) + 0.010;
thirdSetTimeSeries = st_time:0.1:25;

x_firstSet = firstSetTimeSeries * 10;
x_secondSet = [laneChangeMeas(:).x] + 100;
x_thirdSet = thirdSetTimeSeries * 10;

y_firstSet = firstSetTimeSeries * 0;
y_secondSet = [laneChangeMeas(:).y]';
y_thirdSet = thirdSetTimeSeries * 0 + y_secondSet(end);

completeTimeSeries = [firstSetTimeSeries(:); secondSetTimeSeries(:); thirdSetTimeSeries(:)];
completeYSeries = [y_firstSet(:); y_secondSet(:); y_thirdSet(:)];
completeXSeries = [x_firstSet(:); x_secondSet(:); x_thirdSet(:)];

plot (completeXSeries, completeYSeries);

figure(2);
plot(completeTimeSeries, completeXSeries);

figure(3);
plot(completeTimeSeries, completeYSeries);

