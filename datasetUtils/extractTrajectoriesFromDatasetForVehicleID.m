function [trajectories_x, trajectories_y, time_s, vehicle_T] = extractTrajectoriesFromDatasetForVehicleID(T, vehicle_id)
    vehicle_indices = find(T.Vehicle_ID == vehicle_id);
    vehicle_T = T(vehicle_indices, :);
    
    %following minimas and maximas to invert the coordinates to our
    %reference frame
%     minimum_local_y = min( T.Local_X);  % X in the dataset is lateral remember, Y in dataset is long.
    maximum_local_y = max( T.Local_X);
    
    trajectories_x = vehicle_T.Local_Y * 0.3048;
    trajectories_y = (maximum_local_y - vehicle_T.Local_X) * 0.3048;
    time_s = (vehicle_T.Global_Time - vehicle_T.Global_Time(1))/1000;   
end
