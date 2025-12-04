%% Unit test for fcn_LiDARPoseEstimation_FitSphereTarget
% This script tests the fcn_LiDARPoseEstimation_FitSphereTarget function.
% Update inputs and verification logic as needed.
% -------------------------------------------------------------------------
% Purpose:
%   Use robust RANSAC and least squares fitting to extract 3D sphere
%   centers and radii from each LiDAR scan using the initial transformation.
%
% Inputs:
%   - example_calibration_datasets
%   - Initial transformation matrix
%   - Target parameter info
% -------------------------------------------------------------------------

%% Use select dataset and scan to view
dataset_to_view = 1;
