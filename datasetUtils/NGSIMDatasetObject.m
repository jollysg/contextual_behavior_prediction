classdef NGSIMDatasetObject
    properties
        dataset
        vehicle_ids
        vehicles
        
        vehicleIDsWithLaneChange
        vehicleIDsWithoutLaneChange
        
        lane_ids
    end
    
    methods
        function obj = NGSIMDatasetObject(dataset_matfile_name)
            % Load dataset and sort it into vehicles with and without lane
            % changes
            if nargin == 0
                 % read NGSIM trajectory data:
                % read from file: trajectories-0400-0415.csv
                               
                % for reading the table
                obj.dataset = readtable('trajectories-0400-0415.csv');
            else
                obj.dataset = readtable(dataset_matfile_name);
            end
            
            % Extract unique vehicle IDs
            obj.vehicle_ids = unique(obj.dataset.Vehicle_ID);

            obj.vehicleIDsWithLaneChange = [];
            obj.vehicleIDsWithoutLaneChange = [];
%             obj.vehiclesWithLaneChange = VehicleInfo.empty;
%             obj.vehiclesWithoutLaneChange = VehicleInfo.empty;
            obj.vehicles = VehicleInfo.empty;
            
            i = 1;
            j = 1;
            k = 1;

            for j = 1: length(obj.vehicle_ids)
                vehicle_id = obj.vehicle_ids(j);
                vehicle = VehicleInfo(obj.dataset, vehicle_id);
                obj.vehicles(j) = vehicle;
                j = j+1;
                % TODO: Insert vehicle object here
                if vehicle.numberOfLanesOccupied > 1
                    obj.vehicleIDsWithLaneChange(i, 1) = vehicle_id;
                    obj.vehicleIDsWithLaneChange(i, 2) = vehicle.numberOfLanesOccupied;
                    i = i+1;
                else
                    obj.vehicleIDsWithoutLaneChange(k, 1) = vehicle_id;
                    obj.vehicleIDsWithoutLaneChange(k, 2) = vehicle.numberOfLanesOccupied;
                    k = k+1;
                end
            end
            
            obj.lane_ids = unique(obj.dataset.Lane_ID);
        end
        
        function info = getVehicleInfoForID(obj, vehicle_id)
            index = find(obj.vehicle_ids == vehicle_id);
            info = obj.vehicles(index);
        end
        
        function plotVehiclesWithLaneChanges(obj)
            scatter (obj.vehicleIDsWithLaneChange(:,1), obj.vehicleIDsWithLaneChange(:,2))
        end
        
        function stats = findLateralStatsFromDataSetForVehicleId(obj, vehicle_id)
        %   return matrix order - min_y, max_y, mean_y, var_y, stddev_y, lane_no,
        %   number of lanes, vehicle ID
            vehicle_info = obj.getVehicleInfoForID(vehicle_id);
            stats = LateralStats(vehicle_info);
        end
        
    end
            
end
