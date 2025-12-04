function fcn_LiDARPoseEstimation_visualizeRadiusError(...
    fitted_radii, measured_radii, fig_num)
% fcn_LiDARPoseEstimation_visualizeRadiusError
% Visualizes fitted radius error across different target radii
%
% INPUTS:
%   fitted_radii: Nx1
%       Estimated radii from LiDAR fitting (in meters)
%   measured_radii: Nx1
%       Ground truth radii (in meters)
%   fig_num: scalar
%       Figure number for plotting
%
% OUTPUTS:
%   A boxplot showing radius fitting errors grouped by radius size

% Author: Xinyu Cao
% Revised: 2025-07-18

%% Check inputs
if nargin < 3
    fig_num = 910;
end

if length(fitted_radii) ~= length(measured_radii)
    error('Input lengths do not match.');
end

%% Compute error
radius_errors = fitted_radii - measured_radii;
radius_errors_mm = radius_errors * 1000;  % convert to mm
radius_labels_mm = measured_radii * 1000; % in mm

%% Organize data for boxplot
unique_radii = unique(radius_labels_mm);
grouped_labels = [];

for i = 1:length(unique_radii)
    radius_i = unique_radii(i);
    idx_i = abs(radius_labels_mm - radius_i) < 1e-3;
    grouped_labels = [grouped_labels; repmat(radius_i, sum(idx_i), 1)];
end

%% Plot
figure(fig_num); clf;
boxplot(radius_errors_mm, grouped_labels, ...
    'Symbol','r.','OutlierSize',20);
xlabel('Target Radius [mm]');
ylabel('Radius Fitting Error [mm]');
title('LiDAR Fitting Radius Error by Target Size');
grid on;

end
