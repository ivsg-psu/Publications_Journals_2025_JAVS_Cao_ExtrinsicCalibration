function fcn_LiDARPoseEstimation_compareMatchedPoints(...
    ground_truth_points, ...
    predicted_points, ...
    varargin)
% fcn_LiDARPoseEstimation_compareMatchedPoints
% Visualizes the comparison between predicted points (e.g., detected or transformed)
% and reference ground truth using KD-tree matching. Computes per-group residual
% statistics and optionally plots 3D error distribution.
%
% INPUTS:
%   ground_truth_points: Nx3
%       Reference points (e.g., hand-labeled GPS lane markers in ENU)
%
%   predicted_points: Mx3 or 1xK cell
%       Points to be validated (e.g., transformed LiDAR markers)
%
%   varargin:
%       fig_num: (optional) figure number
%       sigma_scale: (optional) the scale of sigma
%
%
% OUTPUTS:
%   None. Displays offset scatter plot and computes per-cluster error metrics.
%
% Author: Xinyu Cao
% Revised: 2025-07-20

%% Parse inputs
fig_num = -1;
if nargin >= 3
    fig_num = varargin{1};
end
flag_do_plot = (fig_num >= 1);

sigma_scale = 1;
if nargin >= 4
    sigma_scale = varargin{2};
end

assert(size(ground_truth_points, 2) == 3, 'Ground truth points must be Nx3');

%% Match predicted points to ground truth via KD-Tree
kdtree = KDTreeSearcher(ground_truth_points);
num_points = size(ground_truth_points, 1);

% Prepare result containers
ave_matched_points_array = [];
ave_dists = NaN(num_points, 1);
std_dists = NaN(num_points, 1);
max_dists = NaN(num_points, 1);
min_dists = NaN(num_points, 1);
X_offsets = [];
Y_offsets = [];
Z_offsets = [];

if iscell(predicted_points)
    for idx_scan = 1:length(predicted_points)
        curr_points = predicted_points{idx_scan};
        % assert(size(curr_points,2)==3, 'Each cell in predicted_points must be Mx3');

        % KD match
        [idxs_match, ~] = knnsearch(kdtree, curr_points);

        for idx_point = 1:num_points
            matched_ground_truth = ground_truth_points(idx_point, :);
            matched_idx = (idxs_match == idx_point);
            matched_pts = curr_points(matched_idx, :);

            if isempty(matched_pts)
                continue;
            end

            ave_matched_point = mean(matched_pts, 1);
            ave_matched_points_array = [ave_matched_points_array; ave_matched_point];

            rel_offsets = matched_pts - matched_ground_truth;
            dist_vals = pdist2(matched_ground_truth, matched_pts, 'euclidean').';

            ave_dists(idx_point) = mean(dist_vals);
            std_dists(idx_point) = std(dist_vals);
            max_dists(idx_point) = max(dist_vals);
            min_dists(idx_point) = min(dist_vals);

            rel_offset_avg = mean(rel_offsets, 1);
            X_offsets = [X_offsets; rel_offset_avg(1)];
            Y_offsets = [Y_offsets; rel_offset_avg(2)];
            Z_offsets = [Z_offsets; rel_offset_avg(3)];
        end
    end
else
    assert(size(predicted_points, 2) == 3, 'Predicted points must be Mx3');
    [idxs_match, ~] = knnsearch(kdtree, predicted_points);

    for idx_point = 1:num_points
        matched_ground_truth = ground_truth_points(idx_point, :);
        matched_idx = (idxs_match == idx_point);
        matched_pts = predicted_points(matched_idx, :);

        if isempty(matched_pts)
            continue;
        end

        ave_matched_point = mean(matched_pts, 1);
        ave_matched_points_array = [ave_matched_points_array; ave_matched_point];

        rel_offsets = matched_pts - matched_ground_truth;
        dist_vals = pdist2(matched_ground_truth, matched_pts, 'euclidean').';

        ave_dists(idx_point) = mean(dist_vals);
        std_dists(idx_point) = std(dist_vals);
        max_dists(idx_point) = max(dist_vals);
        min_dists(idx_point) = min(dist_vals);

        rel_offset_avg = mean(rel_offsets, 1);
        X_offsets = [X_offsets; rel_offset_avg(1)];
        Y_offsets = [Y_offsets; rel_offset_avg(2)];
        Z_offsets = [Z_offsets; rel_offset_avg(3)];
    end
end

%% Plot 3D residual distribution if enabled
if flag_do_plot
    figure(fig_num); clf;
    hold on; grid on; axis equal;
    xlabel('X-axis [m]'); ylabel('Y-axis [m]'); zlabel('Z-axis [m]');
    view(3);

    % Coordinate arrows
    arrow_length = 0.1;
    quiver3(-arrow_length/2, 0, 0, arrow_length, 0, 0, 'r', 'LineWidth', 2); % X
    quiver3(0, -arrow_length/2, 0, 0, arrow_length, 0, 'g', 'LineWidth', 2); % Y
    quiver3(0, 0, -arrow_length/2, 0, 0, arrow_length, 'b', 'LineWidth', 2); % Z

    % Residual scatter
    offset_norms = vecnorm([X_offsets, Y_offsets, Z_offsets], 2, 2);
    scatter3(X_offsets, Y_offsets, Z_offsets, 30, offset_norms, 'filled');
    % Get the original turbo colormap
    cmap = turbo(256);

    % Find the index with maximum green component
    [~, green_idx] = max(cmap(:,2));

    % Extract the segment from green to red
    cmap_green_to_red = cmap(green_idx:end,:);

    % Apply the new colormap
    colormap(cmap_green_to_red);

    % Set color axis starting from 0
    caxis([0 max(offset_norms)]);

    % Add colorbar with label
    c = colorbar;
    c.Label.String = 'Residual error [m]';

    % c.Label.String = 'Residual error [m]';
    % clim([0 max(offset_norms)]);
    % Add ellipsoid showing 3-sigma region
    % Compute mean and std
    offsets_mean = [mean(X_offsets), mean(Y_offsets), mean(Z_offsets)];
    offsets_std = [std(X_offsets), std(Y_offsets), std(Z_offsets)];

    % Create ellipsoid mesh (default unit sphere scaled to 3σ)
    [Xe, Ye, Ze] = ellipsoid(offsets_mean(1), offsets_mean(2), offsets_mean(3), ...
                             sigma_scale*offsets_std(1), sigma_scale*offsets_std(2), sigma_scale*offsets_std(3), 30);
    % Plot ellipsoid
    surf(Xe, Ye, Ze, ...
        'FaceAlpha', 0.2, ...
        'EdgeColor', 'none', ...
        'FaceColor', [0.2 0.6 1.0]);
    
    
    legend('X axis', 'Y axis', 'Z axis', 'Measured points', ...
       sprintf('%d-σ error ellipsoid', sigma_scale), ...
       'Location', 'best');

end

end