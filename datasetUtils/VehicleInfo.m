classdef VehicleInfo < handle
    properties
        vehicleID
        vehicle_dataset
        lanesOccupied
        numberOfLanesOccupied
        trajectories_x
        trajectories_y
        times_s
        
    end
    
    methods
        function obj = VehicleInfo(full_dataset, vehicle_id)
            if nargin == 0
                obj.vehicle_dataset = [];
                obj.lanesOccupied = [];
                obj.numberOfLanesOccupied = 0;
                obj.vehicleID = -1;
                obj.trajectories_x = [];
                obj.trajectories_y = [];
                obj.times_s = [];
            else
                vehicle_indices = find(full_dataset.Vehicle_ID == vehicle_id);
                obj.vehicle_dataset = full_dataset(vehicle_indices, :);
                obj.lanesOccupied = unique(obj.vehicle_dataset.Lane_ID);
                obj.numberOfLanesOccupied = length(obj.lanesOccupied);
                obj.vehicleID = vehicle_id;
                [obj.trajectories_x, obj.trajectories_y, obj.times_s] = obj.extractTrajectoriesFromDatasetForVehicleID(full_dataset);
            end
        end

        function [trajectories_x, trajectories_y, time_s] = extractTrajectoriesFromDatasetForVehicleID(obj, full_dataset)

            vehicle_data = obj.vehicle_dataset;
            %following minimas and maximas to invert the coordinates to our
            %reference frame
            % Lane coordinates are in feet (0.3048 meter). The coordinates are from the
            % left most edge of the section. So lane 1 is the left most lane, lane 6-7
            % is the right most. So the coordinate frame for y needs to be inverted.
            % Further, the Y in the dataset is longitudinal coordinate, x is lateral. 

            % minimum_local_y = min( T.Local_X);  % X in the dataset is lateral remember, Y in dataset is long.
            maximum_local_y = max( full_dataset.Local_X);

            trajectories_x = vehicle_data.Local_Y * 0.3048;
            trajectories_y = (maximum_local_y - vehicle_data.Local_X) * 0.3048;
            time_s = (vehicle_data.Global_Time - vehicle_data.Global_Time(1))/1000;   
        end
        
        function trajectories_v = getVelocityTrajectoriesForVehicle(obj)
            trajectories_v = obj.vehicle_dataset.v_Vel * 0.3048;
        end
        
        function plotLaneOccupancy(obj)
            plot(obj.times_s, obj.vehicle_dataset.Lane_ID);
        end
        
        function plotYTrajectory(obj)
            plot(obj.times_s, obj.trajectories_y);
        end
        
        function plotXTrajectory(obj)
            plot(obj.times_s, obj.trajectories_x);
        end
        
        function plotXYTrajectory(obj)
            plot(obj.trajectories_x, obj.trajectories_y);
        end
    end
end