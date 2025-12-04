function T_final = fcn_LiDARPoseEstimation_iterativePointRegistration(...
    sourcePoints, targetPoints, sphereRadius, referenceRadius, max_iterations)
% fcn_LiDARPoseEstimation_iterativePointRegistration
% Iteratively registers matched 3D point pairs using weighted SVD.
%
% FORMAT:
%   [T_final, inlier_indices] = fcn_LiDARPoseEstimation_iterativePointRegistration(...)
%
% INPUTS:
%   fittedCenters: Nx3 array of detected sphere centers in LiDAR frame
%   targetCenters: Nx3 array of known sphere centers in GPS frame
%   sphereRadii:   Nx1 array of estimated radii from LiDAR fitting
%   referenceRadii:Nx1 array of ground truth radii
%   GPSFrontENU, GPSLeftRearENU, GPSRightRearENU: Nx3 arrays of GPS readings
%   icp_dist_thresh: scalar, distance threshold to stop iteration
%   max_iterations: scalar, maximum number of iterations
%   k_inliers: scalar, MAD scaling for inlier filtering
%
% OUTPUTS:
%   T_final: final transformation (SE(3) object)
%   inlier_indices: indices of final inlier point pairs
%
% Author: Xinyu Cao
% Date: 2025-07-16

% Initialize inliers
source_valid = sourcePoints;
target_valid = targetPoints;
radius_valid = sphereRadius;
radius_ref_valid = referenceRadius;
% GPSFrontENU_inliers = GPSFrontENU;
% GPSLeftRearENU_inliers = GPSLeftRearENU;
% GPSRightRearENU_inliers = GPSRightRearENU;
previous_dist_ave = inf;

% Begin iteration
for iter = 1:max_iterations
    % Compute weights based on radius residuals
    W_array = rescale(1 - abs(radius_valid - radius_ref_valid) ./ radius_ref_valid);
    
    % Fit transformation using weighted SVD
    T_current = fcn_LiDARPoseEstimation_FitTransformationSVD(source_valid, target_valid,W_array);
    
    % Transform LiDAR centers
    fitted_transformed = T_current.transform(source_valid);
    
    % Compute distances
    dist_vec = vecnorm(fitted_transformed - target_valid, 2, 2);
    dist_ave = mean(dist_vec);
    scaled_MAD = 1.4826 * median(abs(dist_vec - median(dist_vec)));
    delta_dist = abs(dist_ave - previous_dist_ave);
    dist_thresh = median(dist_vec) + 3 * scaled_MAD;
    previous_dist_ave = dist_ave;

    % Check convergence
    if delta_dist <= 1E-6
        fprintf("Final error = %.6f after %d iterations.\n", dist_ave, iter);
        break;
    else
        fprintf("Iteration %d, fitting error = %.6f\n", iter, dist_ave);
        
        % Select valid pairs of points
        valid_mask = dist_vec <= dist_thresh;
        source_valid = source_valid(valid_mask,:);
        target_valid = target_valid(valid_mask,:);
        radius_valid = radius_valid(valid_mask,:);
        radius_ref_valid = radius_ref_valid(valid_mask,:);
        % GPSFrontENU_inliers = GPSFrontENU_inliers(valid_mask,:);
        % GPSLeftRearENU_inliers = GPSLeftRearENU_inliers(valid_mask,:);
        % GPSRightRearENU_inliers = GPSRightRearENU_inliers(valid_mask,:);
    end
end
% Final output
T_final = T_current;

end
