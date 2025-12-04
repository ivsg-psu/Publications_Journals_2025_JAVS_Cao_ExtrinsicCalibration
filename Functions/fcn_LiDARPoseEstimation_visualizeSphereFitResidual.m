function fcn_LiDARPoseEstimation_visualizeSphereFitResidual(...
    targets_parameters, measuredRadii_all, fittedError_all, fig_num)
% fcn_LiDARPoseEstimation_visualizeResidualBoxplot
% Generates boxplots of residual errors for fitted spheres,
% grouped by target radius.
%
% INPUTS:
%   targets_parameters: Mx4 array
%       Each row: [X, Y, Z, diameter]
%   measuredRadii_all: Nx1 vector
%       Measured radii of all fitted targets
%   fittedError_all: Nx1 vector
%       Residual fitting error for each fitted target
%   fig_num: scalar
%       Figure number to use for plotting
%
% Author: Xinyu Cao
% Revised: 2025-07-18

%% Initialize
N_targets = size(targets_parameters,1);
ResidualError_array = [];
target_radius_labels_array = [];

%% Accumulate residuals grouped by target radius
for idx_target = 1:N_targets
    target_radius  = targets_parameters(idx_target,4) / 2; % [m]
    tf_current_target = (measuredRadii_all == target_radius);
    residual_error = fittedError_all(tf_current_target);    
    % Optional
    % residual_error = rmoutliers(residual_error);
    % Stack values
    ResidualError_array = [ResidualError_array; residual_error];
    target_radius_label = repmat(target_radius * 1000, size(residual_error)); % [mm]
    target_radius_labels_array = [target_radius_labels_array; target_radius_label];
end

%% Plot boxplot
figure(fig_num); clf;
boxplot(ResidualError_array * 1000, target_radius_labels_array, ...
    'Symbol','r.','OutlierSize',20);
xlabel('Target Radius [mm]');
ylabel('Residuals [mm]');
grid on;
end
