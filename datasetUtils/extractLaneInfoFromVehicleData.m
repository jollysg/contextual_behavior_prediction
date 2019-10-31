function [laneChanges, numberOfLanesOccupancy] = extractLaneInfoFromVehicleData(vehicle_T)
    laneChanges = unique(vehicle_T.Lane_ID);
    numberOfLanesOccupancy = length(laneChanges);
end