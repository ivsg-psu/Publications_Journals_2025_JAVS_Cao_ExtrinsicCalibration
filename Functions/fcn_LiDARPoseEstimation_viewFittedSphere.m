function fcn_LiDARPoseEstimation_viewFittedSphere(...
    dataset, T_LiDAR_to_RearRightGPS, targets_parameters, idx_scan_to_visualize, fig_num, varargin)
% fcn_LiDARPoseEstimation_viewFittedSphere
% Visualizes fitted sphere targets in LiDAR scan and their fitting results,
% with transform option for viewing in LiDAR / GPS / ENU frame.
%
% FORMAT:
%   fcn_LiDARPoseEstimation_viewFittedSphere(dataset, T_LiDAR_to_RearRightGPS, ...
%       targets_parameters, idx_scan_to_visualize, fig_num, (view_frame), (start_idx, N_targets))
%
% INPUTS:
%   dataset: struct with fields:
%       - LiDAR_Velodyne_Rear.PointCloud
%       - GPS_SparkFun_Front_ENU, *_LeftRear_ENU, *_RightRear_ENU
%
%   T_LiDAR_to_RearRightGPS: se3 object
%       Transform from LiDAR to rear-right GPS
%
%   targets_parameters: Mx4
%       Each row is [X Y Z diameter] of a target sphere in ENU
%
%   idx_scan_to_visualize: scalar
%       Index of LiDAR scan to visualize
%
%   fig_num: scalar
%       Figure number
%
%   (OPTIONAL) view_frame: string
%       Frame to view: 'LiDAR' (default), 'GPS', or 'ENU'
%
%   (OPTIONAL) target_range: 1x2
%       [start_idx, number_of_targets_to_show]
%
% Author: Xinyu Cao
% Revised: 2025-07-18

%% Parse optional inputs
view_frame = 'LiDAR';
if nargin >= 6
    view_frame = varargin{1};
end

start_target_idx = 1;
N_targets = size(targets_parameters,1);
if nargin >= 7
    start_target_idx = varargin{2}(1);
    N_targets = varargin{2}(2);
end

flags.flag_do_debug = 0;

%% Extract point cloud and GPS
ptCloud_cell = dataset.LiDAR_Velodyne_Rear.PointCloud;
ptCloud_current = ptCloud_cell{idx_scan_to_visualize};

GPS_Front = dataset.GPS_SparkFun_Front_ENU(idx_scan_to_visualize,:);
GPS_LeftRear = dataset.GPS_SparkFun_LeftRear_ENU(idx_scan_to_visualize,:);
GPS_RightRear = dataset.GPS_SparkFun_RightRear_ENU(idx_scan_to_visualize,:);

%% Compute transforms
M_GPS_to_ENU = fcn_Transform_CalculateTransformation_RearRightGPSToENU(...
    GPS_Front, GPS_LeftRear, GPS_RightRear);
T_GPS_to_ENU = se3(M_GPS_to_ENU);
T_LiDAR_to_ENU = T_GPS_to_ENU * T_LiDAR_to_RearRightGPS;
T_ENU_to_LiDAR = inv(T_LiDAR_to_ENU);
T_LiDAR_to_GPS = inv(T_LiDAR_to_RearRightGPS);
T_ENU_to_GPS = inv(T_GPS_to_ENU);

%% Initialize
Target_Center_ENU = targets_parameters(:,1:3);
target_diameters = targets_parameters(:,4);
xyz_limits = [inf -inf; inf -inf; inf -inf];  % for axis adjustment

%% Start plotting
figure(fig_num); clf;

% Transform once at the end
switch lower(view_frame)
    case 'lidar'
        ptCloud_transformed = ptCloud_current;
        Target_Center_transformed = T_ENU_to_LiDAR.transform(Target_Center_ENU);
    case 'gps'
        ptCloud_transformed = T_LiDAR_to_GPS.transform(ptCloud_current(:,1:3));
        ptCloud_transformed = [ptCloud_transformed, ptCloud_current(:,4:end)];
        Target_Center_transformed = T_ENU_to_GPS.transform(Target_Center_ENU);
    case 'enu'
        ptCloud_transformed = T_LiDAR_to_ENU.transform(ptCloud_current(:,1:3));
        ptCloud_transformed = [ptCloud_transformed, ptCloud_current(:,4:end)];
        Target_Center_transformed = Target_Center_ENU;
    otherwise
        error('Unsupported view_frame. Use ''lidar'', ''gps'', or ''enu''.');
end

% Plot point cloud
scatter3(ptCloud_transformed(:,1), ptCloud_transformed(:,2), ptCloud_transformed(:,3), ...
    10, 'k', 'filled');
hold on;

%% Iterate over sphere targets
N_samples = 4;
outliers_ratio = 0.5;

for idx_target = start_target_idx:N_targets
    center = Target_Center_transformed(idx_target,:);
    radius = target_diameters(idx_target)/2;

    % Define ROI range
    x_rng = [center(1) - 1.5*radius, center(1) + 1.5*radius];
    y_rng = [center(2) - 1.5*radius, center(2) + 1.5*radius];
    z_rng = [center(3) - 1.5*radius, center(3) + 1.5*radius];
    radius_range = [radius-0.03, radius+0.03];

    % Select ROI points
    pc_roi = fcn_LiDARPoseEstimation_filterPointInXYZ(ptCloud_transformed, x_rng, y_rng, z_rng);
    inlier_ratio = 1 - outliers_ratio;
    % N_iter = round(log(1 - 0.99) / log((1 - o)^N_samples));
    N_iter = round(log(1 - 0.99) / log(1 - inlier_ratio^N_samples));
    N_iter = max([50 N_iter]);
    % Inlier identification
    [tf_inliers, outliers_ratio] = fcn_LiDARPoseEstimation_identifyInliersWithRANSAC(...
        pc_roi, N_samples, N_iter, radius_range);

    pc_inliers = pc_roi(tf_inliers,:);

    % Least-squares fitting
    [C_fit, ~, ~] = fcn_LiDARPoseEstimation_FitSphere_Algebraic(pc_inliers(:,1:3));

    % Plotting
    scatter3(Target_Center_transformed(:,1), Target_Center_transformed(:,2), Target_Center_transformed(:,3), 100, 'm', '*','LineWidth',3);
    scatter3(pc_inliers(:,1), pc_inliers(:,2), pc_inliers(:,3), 20, 'b', 'filled');
    scatter3(C_fit(1), C_fit(2), C_fit(3), 100, 'g', 'x','LineWidth',3);
    
    % Update limits
    xyz_limits(1,:) = [min(x_rng(1), xyz_limits(1,1)), max(x_rng(2), xyz_limits(1,2))];
    xyz_limits(2,:) = [min(y_rng(1), xyz_limits(2,1)), max(y_rng(2), xyz_limits(2,2))];
    xyz_limits(3,:) = [min(z_rng(1), xyz_limits(3,1)), max(z_rng(2), xyz_limits(3,2))];
end

%% Final plot format
axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
legend('Road surface', 'GPS measured sphere center','Identified inliers for sphere targets','Fitted sphere center','location','best');
% title(sprintf('LiDAR Targets in %s Frame - Scan %d', upper(view_frame), idx_scan_to_visualize));
grid on;

xyz_limits(1:2,1) = xyz_limits(1:2,1) - 1;
xyz_limits(1:2,2) = xyz_limits(1:2,2) + 1;
xyz_limits(3,1) = xyz_limits(3,1) - 0.5;
xyz_limits(3,2) = xyz_limits(3,2) + 0.5;
axis([xyz_limits(1,:), xyz_limits(2,:), xyz_limits(3,:)]);

end
