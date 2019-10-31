figure(1);
% std_dev vs vehicle ID
scatter (noLaneChangeVehicleStats(:,8), noLaneChangeVehicleStats(:,5))

figure(2);
% mean vs vehicle ID
scatter (noLaneChangeVehicleStats(:,8), noLaneChangeVehicleStats(:,3))

figure(3);
% var vs vehicle ID
scatter (noLaneChangeVehicleStats(:,8), noLaneChangeVehicleStats(:,4))

figure(4);
% lane vs vehicle ID
scatter (noLaneChangeVehicleStats(:,8), noLaneChangeVehicleStats(:,6))

for lane_no = 1:6
    indices = find(noLaneChangeVehicleStats(:,6)==lane_no);
    lane_means = noLaneChangeVehicleStats(indices,3);
    lane_center = mean(lane_means);
    disp("Lane center for lane number " + string(lane_no) + " is: " + string(lane_center));
end