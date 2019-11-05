% clear;

% Get path of this file, and add the parent folder and the subfolders into
% PATH
%   which('<filename>') returns the full path of the file
%   mfilename('fullpath') returns the full path of the file. Without any
%       arguments, this function returns just the filename
%   pwd returns the current working directory of matlab, which can be
%       different from the current file directory
fullFilename = mfilename('fullpath')
% get the parent folder from the full filepath
[filepath, name, ext] = fileparts(fullFilename);

% addpath('./motionModelClasses')
disp('Following files will be added to the path...');
for folder = ["/motionModelClasses", "/mmae_filters", "/filter_classes", "/VehicleParameters", "/datasetUtils"]
    folder_path = append(filepath, folder);
    disp(folder_path)
    addpath(folder_path)
end

x = 0;
vx = 10;
y = 0;
vy = 0;
ay = 0;
rel_x = 0;

X = [x vx y vy]';

%Ts_bp = 0.01;
%Ts_bp = 0.1;
Ts_bp = 1/25;

% covariance of the initial state conditions, i.e. priors
P_init = .001;

%time for the maneuver
% maneuver_length = 150;   % in meters
% tim = maneuver_length / vx;
% lanewidth = 3.5;
% f = 1/(2*maneuver_length);
% w = 2*pi*f;
% A = lanewidth/2 ;   % from middle of the lane to the middle
% y_init = lanewidth/2;         % signifies the starting point in y
% x_init = 0;

%
% vx = 3.81;
% x = 14.6953;
% y = 23.401;
ax = 0;

