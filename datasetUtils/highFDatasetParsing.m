%highD dataset parsing
% first get the tracks loaded into the workspace by running the
% startvisualization.m (or a part of it).

% All the vehicle ids are same as the indices in the tracks struct array.
% So extract the lane info for all the vehicles first.
numOfLaneChanges = [tracks.numLaneChanges];

% find the ones which have lane changes
lane_changes = numOfLaneChanges ~= 0;

% find the indices of vehicles that have lane changes
vehicle_ids_with_lane_changes = find(lane_changes);

vehicles_with_lane_changes = tracks(vehicle_ids_with_lane_changes);

figure(1);
plot([vehicles_with_lane_changes.minDHW]);

% figure(2);
