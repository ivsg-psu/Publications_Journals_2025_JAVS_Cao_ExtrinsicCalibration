function fcn_LiDARPoseEstimation_visualizeCenterDistance(...
    fittedCenters_LiDAR, ...
    measuredCenters_GPS, ...
    targets_parameters, ...
    fig_num)
% fcn_LiDARPoseEstimation_visualizeCenterDistance
% Visualizes center fitting errors (distance) grouped by target radius.
%
% INPUTS:
%   fittedCenters_LiDAR: Nx3 array
%       Fitted sphere centers in LiDAR frame (after transformation)
%   measuredCenters_GPS: Nx3 array
%       Ground truth sphere centers in GPS/ENU frame
%   targets_parameters: Mx4 array
%       Target parameters: [X_ENU, Y_ENU, Z_ENU, diameter]
%   fig_num: scalar
%       Figure number for plotting
%
% OUTPUT:
%   A boxplot showing Euclidean distance errors between fitted and measured centers
%
% Author: Xinyu Cao
% Revised: 2025-07-18

%% Check inputs
if nargin < 4
    fig_num = 901;
end

if size(fittedCenters_LiDAR,1) ~= size(measuredCenters_GPS,1)
    error('Input center arrays must be of the same size.');
end

%% Compute distance error
distance_errors = vecnorm(fittedCenters_LiDAR - measuredCenters_GPS, 2, 2);  % Euclidean distance (in meters)

%% Group by radius
N_targets = size(targets_parameters,1);
ResidualError_array = [];
target_radius_labels_array = [];

for idx_target = 1:N_targets
    target_radius = targets_parameters(idx_target,4) / 2; % in meters
    target_indices = find(abs(vecnorm(measuredCenters_GPS,2,2) - vecnorm(targets_parameters(idx_target,1:3),2)) < 1e-2); % crude match
    if isempty(target_indices)
        continue;
    end
    residual_error = distance_errors(target_indices);    
    ResidualError_array = [ResidualError_array; residual_error];
    radius_labels = repmat(target_radius*1000, size(residual_error)); % mm
    target_radius_labels_array = [target_radius_labels_array; radius_labels];
end

%% Plot
figure(fig_num); clf
boxplot(ResidualError_array*1000, target_radius_labels_array, ...
    'Symbol','r.','OutlierSize',20)
xlabel('Target Radius [mm]')
ylabel('Center Distance Error [mm]')
title('Fitted Center Distance per Target Radius')
grid on

end
