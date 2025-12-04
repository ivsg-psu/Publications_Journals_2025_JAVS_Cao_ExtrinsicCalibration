function fcn_LiDARPoseEstimation_visualizeConstraintSpace(...
    dataset, ...
    T_LiDAR_to_RearRightGPS, ...
    targets_parameters, ...
    idx_scan, ...
    idx_target, ...
    fig_num, ...
    varargin)
% fcn_LiDARPoseEstimation_visualizeConstraintSpace
% Visualizes filtered point cloud data and constraint space cube for a
% specific target at a specific scan, in a chosen coordinate frame.
%
% INPUTS:
%   dataset: struct with fields:
%       - LiDAR_Velodyne_Rear.PointCloud
%       - GPS_SparkFun_Front_ENU, *_LeftRear_ENU, *_RightRear_ENU
%   T_LiDAR_to_RearRightGPS: se3 object
%       Transform from LiDAR to rear-right GPS
%   targets_parameters: Mx4
%       Ground truth sphere centers in ENU and diamsters of the spheres
%   idx_scan: scalar
%       Index of scan to visualize
%   idx_target: scalar
%       Index of target to visualize
%   fig_num: scalar
%       Figure number to use
%   view_frame: string
%       One of {'lidar', 'gps', 'enu'}
%   flag_show_constraint_box: (optional) logical
%       Whether to draw constraint cube (default: true)

% Author: Xinyu Cao
% Revised: 2025-07-18

%% check input arguments
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____                   _
%  |_   _|                 | |
%    | |  _ __  _ __  _   _| |_ ___
%    | | | '_ \| '_ \| | | | __/ __|
%   _| |_| | | | |_) | |_| | |_\__ \
%  |_____|_| |_| .__/ \__,_|\__|___/
%              | |
%              |_|
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

narginchk(6,8);
view_frame = 'LiDAR';
if nargin >= 7
    view_frame = varargin{1};
    if isempty(view_frame)
        view_frame = 'LiDAR';
    end
end

flag_show_constraint_box = 1;
if nargin >= 8
    flag_show_constraint_box = varargin{2};
end
%% Main code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _       
%  |  \/  |     (_)      
%  | \  / | __ _ _ _ __  
%  | |\/| |/ _` | | '_ \ 
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Extract data
ptCloud = dataset.LiDAR_Velodyne_Rear.PointCloud{idx_scan};
GPS_Front = dataset.GPS_SparkFun_Front_ENU(idx_scan,:);
GPS_LeftRear = dataset.GPS_SparkFun_LeftRear_ENU(idx_scan,:);
GPS_RightRear = dataset.GPS_SparkFun_RightRear_ENU(idx_scan,:);
Target_Center_ENU = targets_parameters(:,1:3);
target_diameters = targets_parameters(:,4);
% Compute transforms
M_GPS_to_ENU = fcn_Transform_CalculateTransformation_RearRightGPSToENU(...
    GPS_Front, GPS_LeftRear, GPS_RightRear);
T_GPS_to_ENU = se3(M_GPS_to_ENU);
T_LiDAR_to_ENU = T_GPS_to_ENU * T_LiDAR_to_RearRightGPS;
T_ENU_to_LiDAR = inv(T_LiDAR_to_ENU);
T_ENU_to_GPS = inv(T_GPS_to_ENU);
T_LiDAR_to_GPS = inv(T_LiDAR_to_RearRightGPS);

% Choose frame
switch lower(view_frame)
    case 'lidar'
        ptCloud_trans = ptCloud;
        target_centers = T_ENU_to_LiDAR.transform(Target_Center_ENU);
    case 'gps'
        ptCloud_trans = T_LiDAR_to_GPS.transform(ptCloud(:,1:3));
        ptCloud_trans = [ptCloud_trans, ptCloud(:,4:end)];
        target_centers = T_ENU_to_GPS.transform(Target_Center_ENU);
    case 'enu'
        ptCloud_trans = T_LiDAR_to_ENU.transform(ptCloud(:,1:3));
        ptCloud_trans = [ptCloud_trans, ptCloud(:,4:end)];
        target_centers = Target_Center_ENU;
    otherwise
        error('Unsupported view_frame: %s', view_frame);
end

%% Define constraint region
tgt_center = target_centers(idx_target,:);
tgt_radius = target_diameters(idx_target)/2;
scale = 1.5;
x_rng = [tgt_center(1)-scale*tgt_radius, tgt_center(1)+scale*tgt_radius];
y_rng = [tgt_center(2)-scale*tgt_radius, tgt_center(2)+scale*tgt_radius];
z_rng = [tgt_center(3)-scale*tgt_radius, tgt_center(3)+scale*tgt_radius];

% Extract ROI
pc_roi = fcn_LiDARPoseEstimation_filterPointInXYZ(ptCloud_trans, x_rng, y_rng, z_rng);

%% Plot
figure(fig_num); clf;
hold on;

% Plot filtered point cloud
scatter3(pc_roi(:,1), pc_roi(:,2), pc_roi(:,3), 20, [0.8500 0.3250 0.0980], 'filled');

% Optionally draw constraint cube
if flag_show_constraint_box
    cubeMin = [x_rng(1), y_rng(1), z_rng(1)];
    cubeMax = [x_rng(2), y_rng(2), z_rng(2)];
    vertices = [...
        cubeMin;
        cubeMax(1), cubeMin(2), cubeMin(3);
        cubeMax(1), cubeMax(2), cubeMin(3);
        cubeMin(1), cubeMax(2), cubeMin(3);
        cubeMin(1), cubeMin(2), cubeMax(3);
        cubeMax(1), cubeMin(2), cubeMax(3);
        cubeMax;
        cubeMin(1), cubeMax(2), cubeMax(3)];
    faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];

    patch('Vertices', vertices, 'Faces', faces, ...
        'FaceColor', [0.3 0.8 0.6], 'FaceAlpha', 0.2, ...
        'EdgeColor', 'k', 'LineWidth', 1.2);
end

axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');

if flag_show_constraint_box
    legend('Filtered point cloud data', 'Constraint space');
else
    legend('Filtered point cloud data');
end
title(sprintf('Constraint Space - Scan %d, Target %d (%s frame)', ...
    idx_scan, idx_target, upper(view_frame)));
grid on;
view(3);
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____       _
%  |  __ \     | |
%  | |  | | ___| |__  _   _  __ _
%  | |  | |/ _ \ '_ \| | | |/ _` |
%  | |__| |  __/ |_) | |_| | (_| |
%  |_____/ \___|_.__/ \__,_|\__, |
%                            __/ |
%                           |___/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
