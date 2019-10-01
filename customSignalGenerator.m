load('/home/jasprit/Documents/research/Behavior_identification_for_maneuver/ground_truth_LC.mat');

firstSetTimeSeries = 0:0.01:9.990;
secondSetTimeSeries = ground_truth_LC50.Time + 10;
thirdSetTimeSeries = 15.010:0.01:20;

x_firstSet = firstSetTimeSeries * 10;
x_secondSet = ground_truth_LC50.Data(:,1) + 100;
x_thirdSet = thirdSetTimeSeries * 10;

y_firstSet = firstSetTimeSeries * 0;
y_secondSet = ground_truth_LC50.Data(:,3);
y_thirdSet = thirdSetTimeSeries * 0 + y_secondSet(end);

completeTimeSeries = [firstSetTimeSeries(:); secondSetTimeSeries(:); thirdSetTimeSeries(:)];
completeYSeries = [y_firstSet(:); y_secondSet(:); y_thirdSet(:)];
completeXSeries = [x_firstSet(:); x_secondSet(:); x_thirdSet(:)];

plot (completeXSeries, completeYSeries);

figure(2);
plot(completeTimeSeries, completeXSeries);

figure(3);
plot(completeTimeSeries, completeYSeries);

