%% Unit test for fcn_LiDARPoseEstimation_visualizeConstraintSpace
% This script tests the fcn_LiDARPoseEstimation_visualizeConstraintSpace function.
% Update inputs and verification logic as needed.

% -------------------------------------------------------------------------
% Purpose:
%   Visualize the constraint space (search region) for one selected target
%   before applying RANSAC fitting. Useful for debugging and confirmation.
%
% Inputs:
%   - Index of dataset and scan to visualize
%   - Target index to inspect
% -------------------------------------------------------------------------

%% Use select dataset and scan to view
dataset_to_view = 1;
idx_scan_to_visualize = 1;
idx_target_to_visualize = 2;
%% View all targets in LiDAR frame
sample_data = example_calibration_datasets{dataset_to_view};
fig_num = 22;
fcn_LiDARPoseEstimation_visualizeConstraintSpace( ...
    sample_data, ...
    M_transform_LiDARVelodyne_to_RearRightGPS, ...
    targets_parameters, ...
    idx_scan_to_visualize, ...
    idx_target_to_visualize, fig_num, [], 1);

%% View a specific target in LiDAR frame
sample_data = example_calibration_datasets{dataset_to_view};
fig_num = 22;
fcn_LiDARPoseEstimation_visualizeConstraintSpace( ...
    sample_data, ...
    M_transform_LiDARVelodyne_to_RearRightGPS, ...
    targets_parameters, ...
    idx_scan_to_visualize, ...
    idx_target_to_visualize, fig_num, [], 0);