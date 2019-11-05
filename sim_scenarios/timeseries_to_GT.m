% timeseries to ground truth conversion

% read time from the timeseries object
simtime = out.long_laneChange_GT.trajectories_x.Time;

groundTruth = struct.empty(0,length(simtime));
x_traj = out.long_laneChange_GT.trajectories_x.Data(:);
y_traj = out.long_laneChange_GT.trajectories_y.Data(:);

trueMeasurements = [x_traj';y_traj'];

for i = 1:length(simtime)
    t = simtime(i);
    %Following code adds acceleration of 1m/s after 10 seconds for 1
    %second. As a result, the vehicle accelerates by 1 m/s after this.
    %Comment the following IF condition for a constant velocity run.

    y_gt = trueMeasurements(:,i);
    groundTruth(i).t = t;
    groundTruth(i).gt_states = X_GT;
    groundTruth(i).states = X;
    groundTruth(i).y_tilde = y_gt;
    groundTruth(i).y_gt = y_gt;
end

% truemeas = [groundTruth(:).y_tilde];
% 
% plot(truemeas(1,:), truemeas(2,:));
