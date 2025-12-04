%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Script: script_test_fcn_LiDARPoseEstimation_viewSphereTargets
% Description:
%   Unit test for fcn_LiDARPoseEstimation_viewSphereTargets
%   Loads example LiDAR dataset and visualizes targets projected
%   into the LiDAR frame using an initial transformation
% Author: Xinyu Cao
% Revised: 2025-07-22
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% -------------------------------------------------------------------------
% Purpose:
%   Perform a visual check by projecting ENU ground truth targets into
%   the LiDAR frame using the initial transformation estimate.
%
% Inputs:
%   - One sample dataset from example_calibration_datasets
%   - Initial transformation matrix
%   - Target geometry parameters
% -------------------------------------------------------------------------

%% Use select dataset and scan to view
dataset_to_view = 1;
%% View all targets in LiDAR frame
sample_data = example_calibration_datasets{dataset_to_view};
idx_scan_to_visualize = 12;
fig_num = 21;
fcn_LiDARPoseEstimation_viewSphereTargets( ...
    sample_data, ...
    M_transform_LiDARVelodyne_to_RearRightGPS_Prev, ...
    targets_parameters, ...
    idx_scan_to_visualize, fig_num);

%% View a specific target in LiDAR frame
sample_data = example_calibration_datasets{dataset_to_view};
idx_scan_to_visualize = 12;
target_setting = [3 3]; % [start_target_index, end_target_index]
fig_num = 21;
fcn_LiDARPoseEstimation_viewSphereTargets( ...
    sample_data, ...
    M_transform_LiDARVelodyne_to_RearRightGPS_Prev, ...
    targets_parameters, ...
    idx_scan_to_visualize, fig_num, [], [2 2]);