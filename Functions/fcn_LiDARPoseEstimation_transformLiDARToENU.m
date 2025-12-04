function PointCloud_Transformed_Cell = fcn_LiDARPoseEstimation_transformLiDARToENU(...
    dataset, ...
    PointCloud_Cell, ...
    T_LiDAR_to_RearRightGPS)
% fcn_LiDARPoseEstimation_transformLaneMarkersToENU
% Converts multiple LiDAR-frame lane marker point sets to the ENU frame
% using time-synchronized GPS and LiDAR transformation data.
%
% INPUTS:
%   dataset: struct with fields:
%       - LiDAR_Velodyne_Rear.PointCloud (cell array)
%       - GPS_SparkFun_Front_ENU        (Nx3)
%       - GPS_SparkFun_LeftRear_ENU     (Nx3)
%       - GPS_SparkFun_RightRear_ENU    (Nx3)
%
%   Lane_Marker_Points_Cell: 1xN cell
%       Each cell contains Kx3 lane marker points for scan i in LiDAR frame
%
%   T_LiDAR_to_RearRightGPS: se3 object
%       Rigid transform from LiDAR to rear-right GPS
%
% OUTPUT:
%   lane_marker_points_ENU_all: Mx3
%       Aggregated lane marker points in ENU frame from all scans
%
% Author: Xinyu Cao
% Revised: 2025-07-20

%% Initialize
N_scans = length(PointCloud_Cell);
lane_marker_points_ENU_all = [];
PointCloud_Transformed_Cell = cell(N_scans,1);
%% Loop through all scans
for idx_scan = 1:N_scans
    % Skip empty scans
    if isempty(PointCloud_Cell{idx_scan})
        continue;
    end

    % Get current GPS data
    GPS_Front     = dataset.GPS_SparkFun_Front_ENU(idx_scan,:);
    GPS_LeftRear  = dataset.GPS_SparkFun_LeftRear_ENU(idx_scan,:);
    GPS_RightRear = dataset.GPS_SparkFun_RightRear_ENU(idx_scan,:);

    % Compute GPS → ENU transform
    M_GPS_to_ENU = fcn_Transform_CalculateTransformation_RearRightGPSToENU(...
        GPS_Front, GPS_LeftRear, GPS_RightRear);
    T_GPS_to_ENU = se3(M_GPS_to_ENU);

    % Combined LiDAR → ENU transform
    T_LiDAR_to_ENU = T_GPS_to_ENU * T_LiDAR_to_RearRightGPS;

    % Transform lane marker points
    pointcloud_ith_scan = PointCloud_Cell{idx_scan};
    pointcloud_ENU = T_LiDAR_to_ENU.transform(pointcloud_ith_scan(:,1:3));

    % Store
    % lane_marker_points_ENU_all = [lane_marker_points_ENU_all; pointcloud_ENU];
    PointCloud_Transformed_Cell{idx_scan} = pointcloud_ENU;
end

end
