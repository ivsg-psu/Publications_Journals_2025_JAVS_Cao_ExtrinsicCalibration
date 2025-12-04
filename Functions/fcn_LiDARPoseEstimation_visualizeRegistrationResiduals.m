function fcn_LiDARPoseEstimation_visualizeRegistrationResiduals(...
    ground_truth_points, ...
    transformed_points, ...
    fig_num)
% fcn_LiDARPoseEstimation_visualizeRegistrationResiduals
% Compares predicted (fitted or detected) points to ground truth and
% visualizes registration residuals in both histogram and 3D offset form.
%
% INPUTS:
%   ground_truth_points: Nx3
%       Ground truth reference points (e.g., ENU or map coordinates)
%
%   predicted_points: Mx3
%       Points to validate (e.g., transformed LiDAR centers or lane markers)
%
%   fig_num: scalar
%       Figure number for histogram plot
%
% OUTPUT:
%   None (displays residual histogram and 3D scatter of offset vectors)
%
% Author: Xinyu Cao
% Revised: 2025-07-20

%% Input check
if nargin < 3
    fig_num = 100;
end

assert(size(ground_truth_points, 2) == 3, 'Ground truth must be Nx3.');
assert(size(transformed_points, 2) == 3, 'Predicted must be Mx3.');
assert(~isempty(ground_truth_points) && ~isempty(transformed_points), 'Inputs must be non-empty.');

%% Nearest neighbor matching
kdtree = KDTreeSearcher(ground_truth_points);
[idxs_match, ~] = knnsearch(kdtree, transformed_points);
num_points = length(ground_truth_points);
for idx_point = 1:num_points
    matched_ground_truth = ground_truth_points(idx_point,:);
    matched_idx = (idxs_match == idx_point);
    matched_points = transformed_points(matched_idx,:);
    if isempty(matched_points)
        continue;
    end
    offset_vectors = matched_points - matched_ground_truth;
    offset_dists = pdist2(matched_ground_truth, matched_points, 'euclidean').';
    X_offset = offset_vectors(:,1);
    Y_offset = offset_vectors(:,2);
    Z_offset = offset_vectors(:,3);
end
matched_gt = ground_truth_points(idxs_match, :);
matched_pred = transformed_points;

offset_vectors = matched_pred - matched_gt;
offset_norms = vecnorm(offset_vectors, 2, 2);

% Separate components
X_offset = offset_vectors(:,1);
Y_offset = offset_vectors(:,2);
Z_offset = offset_vectors(:,3);

%% Plot 1: Residual histogram
figure(fig_num); clf;
histogram(offset_norms * 1000, 20); % in mm
xlabel('Residual Error [mm]');
ylabel('Frequency');
title(sprintf(['Registration Residuals\nMean = %.2f mm, Std = %.2f mm, Max = %.2f mm, RMSE = %.2f mm'], ...
    mean(offset_norms)*1000, std(offset_norms)*1000, max(offset_norms)*1000, sqrt(mean(offset_norms.^2))*1000));
grid on;

%% Plot 2: Residual vector distribution (3D)
figure(8976); clf;
hold on;
grid on;
xlabel('X-axis'); ylabel('Y-axis'); zlabel('Z-axis');
axis equal;
view(3);

% Draw axes arrows
arrow_length = 0.1;
quiver3(-arrow_length/2, 0, 0, arrow_length, 0, 0, 'r', 'LineWidth', 2); % X
quiver3(0, -arrow_length/2, 0, 0, arrow_length, 0, 'g', 'LineWidth', 2); % Y
quiver3(0, 0, -arrow_length/2, 0, 0, arrow_length, 'b', 'LineWidth', 2); % Z

% Scatter residual vectors
scatter3(X_offset, Y_offset, Z_offset, 30, offset_norms, 'filled');

% Colorbar
c = colorbar;
c.Label.String = 'Residual error [m]';
colormap parula;

legend('X axis', 'Y axis', 'Z axis', 'Offset vectors', 'Location', 'best');
title('3D Residual Error Distribution');

%% Optional summary printout
fprintf('[Registration Residuals] Mean: %.2f mm | Std: %.2f mm | Max: %.2f mm | RMSE: %.2f mm\n', ...
    mean(offset_norms)*1000, std(offset_norms)*1000, max(offset_norms)*1000, sqrt(mean(offset_norms.^2))*1000);

end
