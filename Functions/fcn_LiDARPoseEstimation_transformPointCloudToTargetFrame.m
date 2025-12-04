function transformed_pointCloud = fcn_LiDARPoseEstimation_transformPointCloudToTargetFrame(...
    pointCloud, ...
    T_LiDAR_to_GPS, ...
    target_frame, ...
    varargin)
% fcn_LiDARPoseEstimation_transformPointCloudToTargetFrame
% Transforms a point cloud from LiDAR frame to GPS or ENU frame.
%
% INPUTS:
%   pointCloud: Nx3 or Nx4 matrix
%       Raw LiDAR point cloud (X, Y, Z [, Intensity])
%
%   T_LiDAR_to_GPS: se3 object
%       Rigid-body transformation from LiDAR frame to GPS frame
%
%   target_frame: string
%       Target coordinate frame, one of {'GPS', 'ENU'}
%
%   varargin: optional
%       If target_frame is 'ENU', then varargin{1} should be T_GPS_to_ENU (se3)
%
% OUTPUT:
%   transformed_pointCloud: Nx3 or Nx4 matrix
%       Transformed point cloud in the desired target frame
%
% Author: Xinyu Cao
% Revised: 2025-07-20

%% Input checks
assert(size(pointCloud,2)==3 || size(pointCloud,2)==4, ...
    'Input pointCloud must be Nx3 or Nx4.');
assert(isa(T_LiDAR_to_GPS, 'se3'), ...
    'T_LiDAR_to_GPS must be a se3 object.');
assert(ischar(target_frame) || isstring(target_frame), ...
    'target_frame must be a string.');

target_frame = lower(string(target_frame));

if target_frame == "enu"
    assert(~isempty(varargin), 'T_GPS_to_ENU is required when target_frame is "ENU".');
    T_GPS_to_ENU = varargin{1};
    assert(isa(T_GPS_to_ENU, 'se3'), 'T_GPS_to_ENU must be a se3 object.');
end

%% Step 1: Transform from LiDAR → GPS
XYZ = pointCloud(:,1:3);
XYZ_GPS = T_LiDAR_to_GPS.transform(XYZ);

%% Step 2: If ENU, apply GPS → ENU
switch target_frame
    case "gps"
        XYZ_target = XYZ_GPS;

    case "enu"
        XYZ_target = T_GPS_to_ENU.transform(XYZ_GPS);

    otherwise
        error('Unsupported target_frame: %s. Use "GPS" or "ENU".', target_frame);
end

%% Step 3: Reattach intensity if needed
if size(pointCloud,2)==4
    transformed_pointCloud = [XYZ_target, pointCloud(:,4)];
else
    transformed_pointCloud = XYZ_target;
end

end
