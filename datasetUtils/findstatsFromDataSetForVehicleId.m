function stats = findstatsFromDataSetForVehicleId(T, vehicle_id)
%   return matrix order - min_y, max_y, mean_y, var_y, stddev_y, lane_no,
%   number of lanes, vehicle ID
    vehicle_indices = find(T.Vehicle_ID == vehicle_id);
    vehicle_T = T(vehicle_indices, :);
    
    %following minimas and maximas to invert the coordinates to our
    %reference frame
    minimum_local_y = min( T.Local_X);  % X in the dataset is lateral remember, Y in dataset is long.
    maximum_local_y = max( T.Local_X);
    
%     laneChanges = unique(vehicle_T.Lane_ID);
%     numberOfLanesOccupancy = length(laneChanges);

    % Return order - vector of lane changes, total number of lanes
    [laneChanges, numberOfLanesOccupancy] = extractLaneInfoFromVehicleData(vehicle_T);

    trajectory_x_m = vehicle_T.Local_Y * 0.3048;
    trajectory_y_m = (maximum_local_y - vehicle_T.Local_X) * 0.3048;
    time_s = (vehicle_T.Global_Time - vehicle_T.Global_Time(1))/1000;
    
    % stats - init lane, lane occupancys, 
    stats = [findStatsFromVector(trajectory_y_m), laneChanges(1), numberOfLanesOccupancy, vehicle_id];
end
