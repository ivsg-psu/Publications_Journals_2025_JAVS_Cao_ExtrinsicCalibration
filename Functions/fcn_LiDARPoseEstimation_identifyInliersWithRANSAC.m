function [best_tf_inliers, outliers_ratio] = ...
    fcn_LiDARPoseEstimation_identifyInliersWithRANSAC(...
    LiDAR_pointCloud_inrange, N_samples, iter_max, ...
    radius_range, varargin)
% fcn_LiDARPoseEstimation_identifyInliersWithRANSAC
% Uses RANSAC to identify inlier points and fit a sphere to 3D LiDAR data.
%
% FORMAT:
%   [best_model, best_tf_inliers, outliers_ratio] = ...
%       fcn_LiDARPoseEstimation_identifyInliersWithRANSAC(...
%       LiDAR_pointCloud_inrange, N_samples, iter_max, inliers_thresh, ...
%       dist_thresh_input, target_radius, varargin)
%
% INPUTS:
%   LiDAR_pointCloud_inrange: NxK array
%       Input point cloud, at minimum with XYZ (first 3 cols).
%
%   N_samples: scalar
%       Minimum sample points for a RANSAC model hypothesis.
%
%   iter_max: scalar
%       Maximum RANSAC iterations.
%
%   inliers_thresh: scalar
%       Minimum number of inliers required to accept a model.
%
%   dist_thresh_input: scalar
%       Base distance threshold (e.g. 0.05 m) for inlier classification.
%
%   target_radius: scalar
%       Expected sphere radius [m].
%
%   varargin: (optional)
%       flag_do_debug: scalar
%           
%
% OUTPUTS:
%   best_model: 1x4 vector
%       [Xc, Yc, Zc, R] - fitted sphere model
%
%   best_tf_inliers: Nx1 logical array
%       Inlier mask indicating best-fit inliers
%
%   outliers_ratio: scalar
%       Estimated outlier percentage in the dataset
%
% DEPENDENCIES:
%   fcn_LiDARPoseEstimation_FitSphereLSQ
%   fcn_LiDARPoseEstimation_EvalSphere
%
% Author: Xinyu Cao
% Date: 2025-07-16


%% Input check

narginchk(4,5);

flag_do_debug = 0;

if nargin >=5
    flag_do_debug = varargin{1};
end

%% Debugging setup



% flag_check_inputs = 1;

if flag_do_debug
    st = dbstack;
    fprintf(1, 'STARTING function: %s, in file: %s\n', st(1).name, st(1).file);
end


%% Initialization
best_model = nan(1,4);
best_tf_inliers = false(size(LiDAR_pointCloud_inrange,1),1);
outliers_ratio = nan;
best_score = inf;
best_count_of_inliers = nan;

xyz_data = LiDAR_pointCloud_inrange(:,1:3);
N_points = size(xyz_data,1);
if N_points < 4
    return;
end
N_samples = min([N_samples, N_points]);

%% RANSAC iterations
for idx_iter = 1:iter_max
    idx_sample = randsample(N_points, N_samples);
    sample_pts = xyz_data(idx_sample,:);

 
    [C_sphere, R_sphere, ~] = fcn_LiDARPoseEstimation_FitSphere_Algebraic(sample_pts);
    if R_sphere<radius_range(1) || R_sphere>radius_range(2)
        continue;
    end
    % dist_pts = vecnorm(xyz_data - C_sphere, 2, 2);
    % residual_errors = abs(dist_pts - R_sphere);
    dist2_pts = sum((xyz_data - C_sphere).^2, 2);  % Squared distance
    residual_errors = abs(dist2_pts - R_sphere^2);  % Algebraic residual

    % Robust MAD-based threshold
    k = -1/(sqrt(2)*erfcinv(3/2));
    scaled_MAD = k * median(abs(residual_errors - median(residual_errors)));
    dist_thresh = median(residual_errors) + 3 * scaled_MAD;
    
    tf_inliers = residual_errors <= dist_thresh;
    idxs_inliers = find(tf_inliers);
    num_inliers = numel(idxs_inliers);
    inliers_ratio = num_inliers / N_points;
    outliers_ratio = 1 - inliers_ratio;
    dist2_inliers = dist2_pts(idxs_inliers,:);
    dist_inliers = sqrt(dist2_inliers);
    residual_errors_inliers = abs(dist_inliers - R_sphere);
    score = mean(residual_errors_inliers);
    % If both conditions satisfied
    if inliers_ratio<=0.5 || score>best_score
        continue;
    end
    best_tf_inliers = tf_inliers;
    best_score = score;

end

%% Optional plotting
if flag_do_debug
    fprintf(1, 'ENDING function: %s\n\n', st(1).name);
end

end
