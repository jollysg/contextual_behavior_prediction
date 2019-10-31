classdef LateralStats
%     min_y, max_y, mean_y, var_y, stddev_y, lane_no,
        %   number of lanes, vehicle ID
    properties
        min_y
        max_y
        mean_y
        var_y
        stddev_y
        lane_nos
        numberOfLanes
        vehicleID
        number_of_positions
    end
    
    methods
        function obj = LateralStats(vehicleInfo)
            obj.min_y = min(vehicleInfo.trajectories_y);
            obj.max_y = max(vehicleInfo.trajectories_y);
            obj.mean_y = mean(vehicleInfo.trajectories_y);
            obj.var_y = var(vehicleInfo.trajectories_y);
            obj.stddev_y = std(vehicleInfo.trajectories_y);
            obj.lane_nos = vehicleInfo.lanesOccupied;
            obj.numberOfLanes = vehicleInfo.numberOfLanesOccupied;
            obj.vehicleID = vehicleInfo.vehicleID;
            obj.number_of_positions = length(vehicleInfo.trajectories_y);
        end
    end
end