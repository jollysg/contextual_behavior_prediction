% post highfD dataset initialization, setup.
% MAke sure, tracks is loaded in the workspace. This can be done by running
% the initialization and trackvisualization scripts in HighDdataset
vehicleHD118 = HighDVehicleInfo(tracks, 118);
vehicleHD111 = HighDVehicleInfo(tracks, 111);
vehicleHD99 = HighDVehicleInfo(tracks, 99);

% Has both front and end
vehicleHD167 = HighDVehicleInfo(tracks, 167);
vehicleHD142 = HighDVehicleInfo(tracks, 142);

vehicle = vehicleHD167;
vehicle.calculateXYTrajectories();

x = vehicle.trajectories_x(1);
y = vehicle.trajectories_y(1);
vx = vehicle.trajectories_xvel(1);
vy = vehicle.trajectories_yvel(1);
ax = vehicle.trajectories_xacc(1);
ay = vehicle.trajectories_yacc(1);