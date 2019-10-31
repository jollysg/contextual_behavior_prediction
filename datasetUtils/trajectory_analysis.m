% read NGSIM trajectory data:
% read from file: trajectories-0400-0415.csv

% for reading the table
%T = readtable('trajectories-0400-0415.csv');

% trajectories from MAT, load the table
load('trajectories-0400-0415.mat');


% For time zone conversion
times = datetime(T.Global_Time(:)/1000,'ConvertFrom','posixTime','TimeZone','America/Los_Angeles','Format','dd-MMM-yyyy HH:mm:ss.SSS');


% Extract unique vehicle IDs
vehicles = unique(T.Vehicle_ID);

%finding indices for one vehicle ID in the data
vehicle1 = find(T.Vehicle_ID ==1);

%vehicle x position (lane) against y (road length)
plot(T.Local_Y(vehicle1(1:100)), T.Local_X(vehicle1(1:100)));

% vehicle lane ID against road length
plot(T.Local_Y(vehicle1), T.Lane_ID(vehicle1));

%All the entries for a vehicle (vehicle 2 in this case)
%vehicle2_T = T(vehicle2_indices, :);

vehicleIDsWithLaneChange = [];
vehicleIDsWithoutLaneChange = [];
i = 1;
j = 1;
k = 1;

for j = 1: length(vehicles)
    vehicle_id = vehicles(j);
    j = j+1;
    vehicle_indices = find(T.Vehicle_ID == vehicle_id);
    vehicle_T = T(vehicle_indices, :);
    laneChanges = unique(vehicle_T.Lane_ID);
    numberOfLanesOccupancy = length(laneChanges);
    if numberOfLanesOccupancy > 1
        vehicleIDsWithLaneChange(i, 1) = vehicle_id;
        vehicleIDsWithLaneChange(i, 2) = numberOfLanesOccupancy;
        i = i+1;
    else
        vehicleIDsWithoutLaneChange(k, 1) = vehicle_id;
        vehicleIDsWithoutLaneChange(k, 2) = numberOfLanesOccupancy;
        k = k+1;
    end
    clear vehicle2_indices; clear vehicle_T;
end

%%
%vehicle ID 7 has a lane change

figure(2);
hold on
legendArray = {};
start = 7;
last = 7;
for i = start:last
    vehicle_id = vehicleIDsWithLaneChange(i);
    log_print = sprintf('Showing vehicle id no %f', vehicle_id);
    disp(log_print);
    vehicle_indices = find(T.Vehicle_ID == vehicle_id);
    vehicle_T = T(vehicle_indices, :);
    plot(vehicle_T.Lane_ID);
    legendArray{i-start+1} = sprintf("Vehicle ID %d", vehicle_id);
end
legend(legendArray)
hold off;

figure(3);

plot (vehicle_T.Local_Y, vehicle_T.Local_X);
ylim([0 100]);
xlim([0 1800]);


%%

minimum_local_y = min( T.Local_X);  % X in the dataset is lateral remember, Y in dataset is long.
maximum_local_y = max( T.Local_X);

trajectory_x_m = vehicle_T.Local_Y * 0.3048;
trajectory_y_m = (maximum_local_y - vehicle_T.Local_X) * 0.3048;

% Lane coordinates are in feet (0.3048 meter). The coordinates are from the
% left most edge of the section. So lane 1 is the left most lane, lane 6-7
% is the right most. So the coordinate frame for y needs to be inverted.
% Further, the Y in the dataset is longitudinal coordinate, x is lateral. 
plot (trajectory_x_m, trajectory_y_m);
ylim([0 100 * 0.3048]);
xlim([0 1800 * 0.3048]);

vehicle_times = datetime(vehicle_T.Global_Time(:)/1000,'ConvertFrom','posixTime','TimeZone','America/Los_Angeles','Format','dd-MMM-yyyy HH:mm:ss.SSS');

times_trial = vehicle_times - vehicle_times(1);

time_s = (vehicle_T.Global_Time - vehicle_T.Global_Time(1))/1000;

figure(4);
disp("showing for lane id: " + string(vehicle_id))
plot(vehicle_T.Lane_ID);

%Extract vehicle velocity
vehicle_vel = vehicle_T.v_Vel * 0.3048;

% Extract vehicle acceleration
vehicle_acc = vehicle_T.v_Acc * 0.3048;

% function [min_y, max_y, mean_y, var_y, stddev_y] = findStatsFromVector(data_vector)
%     min_y = min(data_vector);
%     max_y = max(data_vector);
%     mean_y = mean(data_vector);
%     var_y = var(data_vector);
%     stddev_y = std(data_vector);
% end

noLaneChangeVehicleStats = [];
for i = 1:length(vehicleIDsWithoutLaneChange)
    vehicle_id = vehicleIDsWithoutLaneChange(i);
    noLaneChangeVehicleStats(i,:) = findstatsFromDataSetForVehicleId(T, vehicle_id);
end

