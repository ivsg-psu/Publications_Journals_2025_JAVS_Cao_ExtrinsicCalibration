function fcn_LiDARPoseEstimation_plotLaneMarkersENU(...
    dataset, ...
    lane_marker_ENU, ...
    idx_scan, ...
    T_LiDAR_to_RearRightGPS,fig_num)
% fcn_LiDARPoseEstimation_plotLaneMarkersENU
% Plots extracted lane marker points in the ENU frame by transforming 
% them from the LiDAR coordinate frame, using time-synchronized GPS and 
% point cloud data.
%
% INPUTS:
%   dataset: struct with fields:
%       - LiDAR_Velodyne_Rear.PointCloud (cell array)
%       - GPS_SparkFun_Front_ENU        (Nx3)
%       - GPS_SparkFun_LeftRear_ENU     (Nx3)
%       - GPS_SparkFun_RightRear_ENU    (Nx3)
%   lane_marker_points_LiDAR: Kx3
%       Lane marker points in LiDAR coordinate frame
%   idx_scan: scalar
%       Index of scan to visualize
%   T_LiDAR_to_RearRightGPS: se3 object
%       Transformation from LiDAR to rear-right GPS

% Author: Xinyu Cao
% Revised: 2025-07-20

%% Input checks
assert(isfield(dataset, 'LiDAR_Velodyne_Rear') && isfield(dataset.LiDAR_Velodyne_Rear, 'PointCloud'), 'Missing LiDAR data.');
assert(isfield(dataset, 'GPS_SparkFun_Front_ENU'), 'Missing GPS Front data.');
assert(isfield(dataset, 'GPS_SparkFun_LeftRear_ENU'), 'Missing GPS Left Rear data.');
assert(isfield(dataset, 'GPS_SparkFun_RightRear_ENU'), 'Missing GPS Right Rear data.');

%% Extract current scan and GPS data
ptCloud = dataset.LiDAR_Velodyne_Rear.PointCloud{idx_scan};
GPS_Front = dataset.GPS_SparkFun_Front_ENU(idx_scan,:);
GPS_LeftRear = dataset.GPS_SparkFun_LeftRear_ENU(idx_scan,:);
GPS_RightRear = dataset.GPS_SparkFun_RightRear_ENU(idx_scan,:);
% lane_marker_points_LiDAR = Lane_Marker_Points_Cell{idx_scan};
%% Compute ENU transform
M_GPS_to_ENU = fcn_Transform_CalculateTransformation_RearRightGPSToENU(...
    GPS_Front, GPS_LeftRear, GPS_RightRear);
T_GPS_to_ENU = se3(M_GPS_to_ENU);

%% Apply full LiDAR → ENU transform
T_LiDAR_to_ENU = T_GPS_to_ENU * T_LiDAR_to_RearRightGPS;
ptCloud_ENU = T_LiDAR_to_ENU.transform(ptCloud(:,1:3));
% lane_marker_ENU = T_LiDAR_to_ENU.transform(lane_marker_points_LiDAR);

%% Plot
figure(fig_num); clf;
scatter(ptCloud_ENU(:,1), ptCloud_ENU(:,2), 10, ptCloud(:,4), 'filled'); % Intensity coloring
hold on;
scatter(lane_marker_ENU(:,1), lane_marker_ENU(:,2), 50, 'r', 'filled');

xlabel('X-East [m]');
ylabel('Y-North [m]');
legend('LiDAR scan in ENU', 'Handmeasured lane marker', 'Location', 'best');
grid on;
axis equal;
% title(sprintf('Lane Marker Visualization in ENU (Scan %d)', idx_scan));

end
