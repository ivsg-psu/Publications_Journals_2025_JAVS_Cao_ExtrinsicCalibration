function [roll_offset, pitch_offset] = fcn_LiDARPoseEstimation_compareGroundAngle(...
    dataset_struct, ground_truth_points)
% fcn_LiDARPoseEstimation_compareGroundAngle
% Compares estimated road slope (roll/pitch from ground plane normal)
% with ground truth slope from reference normal vector.
%
% FORMAT:
%   [roll_offset, pitch_offset] = ...
%       fcn_LiDARPoseEstimation_compareGroundAngle(...
%       dataset_struct, ground_truth_points)
%
% INPUTS:
%   dataset_struct: struct
%       Must contain:
%       - LiDAR_Velodyne_Rear.PointCloud: 1xN cell array of scans
%
%   ground_truth_points: Nx3 array
%       3D points defining the reference ground plane (e.g. in ENU)
%
% OUTPUTS:
%   roll_offset: Nx1 vector (deg)
%       Difference between scan-estimated and ground truth roll angle
%
%   pitch_offset: Nx1 vector (deg)
%       Difference between scan-estimated and ground truth pitch angle
%
% Author: Xinyu Cao
% Revised: 2025-07-23

%% Check inputs
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
assert(isfield(dataset_struct, 'LiDAR_Velodyne_Rear'), 'Missing LiDAR data field.');
assert(isfield(dataset_struct.LiDAR_Velodyne_Rear, 'PointCloud'), 'Missing PointCloud.');


%% Main function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _       
%  |  \/  |     (_)      
%  | \  / | __ _ _ _ __  
%  | |\/| |/ _` | | '_ \ 
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Extract fields
LiDAR_pointCloud = dataset_struct.LiDAR_Velodyne_Rear.PointCloud;  % 1xN cell
N_frames = size(LiDAR_pointCloud,1);


%% Fit reference ground plane with GPS measurements
[~, ~, ~, normal_vector_ground_truth, ~, ~, ~] = fcn_geometry_fitPlaneLinearRegression(ground_truth_points);

[ground_roll_rad, ground_pitch_rad] = fcn_Internal_calculateAngle(normal_vector_ground_truth);
ground_roll_deg = rad2deg(ground_roll_rad);
ground_pitch_deg = rad2deg(ground_pitch_rad);

% Storage

scan_roll_deg = zeros(N_frames,1);
scan_pitch_deg = zeros(N_frames,1);
%% Main loop
for ith_frame = 1:N_frames
    pointCloud_ith_frame = LiDAR_pointCloud{ith_frame};
    if isempty(pointCloud_ith_frame)

        scan_roll_deg(ith_frame) = NaN;
        scan_pitch_deg(ith_frame) = NaN;
        continue;
    end

    % Extract XYZ
    XYZ = pointCloud_ith_frame(:,1:3);
    ground_points = XYZ;
    % Fit plane to ground points
    if size(ground_points,1) < 10
        scan_roll_deg(ith_frame) = NaN;
        scan_pitch_deg(ith_frame) = NaN;
        continue;
    end

    [~, ~, ~, normal_vector, ~, ~, ~] = fcn_geometry_fitPlaneLinearRegression(ground_points);
    % Compute roll and pitch with normal vector
    [scan_roll_rad, scan_pitch_rad] = fcn_Internal_calculateAngle(normal_vector);
    scan_roll_deg(ith_frame) = rad2deg(scan_roll_rad);
    scan_pitch_deg(ith_frame) = rad2deg(scan_pitch_rad);
    
    
end

%% Compute offset from ground truth
roll_offset = scan_roll_deg - ground_roll_deg;
pitch_offset = scan_pitch_deg - ground_pitch_deg;

end

function [roll_rad, pitch_rad] = fcn_Internal_calculateAngle(normal_vector)
    % Convert normal to roll/pitch
    % Assume Z-up: normal = [nx, ny, nz]
    if normal_vector(3) < 0
        normal_vector = - normal_vector;
    end
    roll_rad  = atan2(normal_vector(2), normal_vector(3));
    pitch_rad = atan2(-normal_vector(1), sqrt(normal_vector(2)^2+normal_vector(3)^2));
end
