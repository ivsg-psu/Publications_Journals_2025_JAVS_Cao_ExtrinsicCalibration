function [fittedCenters_LiDAR, measuredCenters_GPS, ...
          fittedRadii_LiDAR, measuredRadii, fittedError] = ...
         fcn_LiDARPoseEstimation_FitSphereTarget(...
         clean_dataStructure, targets_parameters, ...
         M_transform_LiDAR_to_GPS_previous, N_samples, ...
         LiDAR_accuracy, varargin)
% fcn_LiDARPoseEstimation_FitSphereTarget
% Fits multiple spherical targets from a static LiDAR dataset using RANSAC 
% and least squares refinement. Transforms ground truth sphere centers from 
% ENU to the LiDAR frame using an initial guess and returns estimated sphere 
% positions in the LiDAR frame along with residual errors.
%
% FORMAT:
%   [fittedCenters_LiDAR, measuredCenters_GPS, ...
%    fittedRadii_LiDAR, measuredRadii, fittedError] = ...
%    fcn_LiDARPoseEstimation_FitSphereTarget(...
%       clean_dataStructure, targets_parameters, ...
%       M_transform_LiDAR_to_GPS_previous, N_samples, ...
%       LiDAR_accuracy, varargin)
%
% INPUTS:
%   clean_dataStructure: struct
%       Struct containing LiDAR point cloud scans, in the field:
%         - LiDAR_Velodyne_Rear.PointCloud: cell array of Nx3 point clouds.
%
%   targets_parameters: Mx4 matrix
%       Ground truth parameters of M spherical targets in ENU frame.
%       Each row: [X_ENU, Y_ENU, Z_ENU, Diameter].
%
%   M_transform_LiDAR_to_GPS_previous: 4x4 matrix
%       Initial transformation matrix from LiDAR frame to RearRight GPS.
%
%   N_samples: scalar
%       Number of sample points for RANSAC fitting per scan.
%
%   LiDAR_accuracy: scalar
%       Allowed tolerance (in meters) for sphere radius residuals.
%
%   varargin: (optional)
%       fig_num: scalar
%           Figure number for optional plotting. Set to 0 to disable.
%
% OUTPUTS:
%   fittedCenters_LiDAR: Mx3 array
%       Estimated sphere centers in the LiDAR frame (one per scan).
%
%   measuredCenters_GPS: Mx3 array
%       Ground truth sphere centers transformed to the LiDAR frame.
%
%   fittedRadii_LiDAR: Mx1 vector
%       Estimated radii for each fitted sphere from LiDAR data.
%
%   measuredRadii: Mx1 vector
%       Ground truth sphere radii (half of the input diameters).
%
%   fittedError: Mx1 vector
%       Average residual fitting error (in meters) for each sphere.
%

% Author: Xinyu Cao
% Revised: 2025-07-16
%% check input arguments
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____                   _
%  |_   _|                 | |
%    | |  _ __  _ __  _   _| |_ ___
%    | | | '_ \| '_ \| | | | __/ __|
%   _| |_| | | | |_) | |_| | |_\__ \
%  |_____|_| |_| .__/ \__,_|\__|___/
%              | |
%              |_|
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Inputs

flag_do_debug = 0;

fig_num = 0;
if nargin >= 6
    fig_num = varargin{1};
end
flag_do_plot = 0;
if fig_num >= 1
    flag_do_plot = 1;
end

%% GPS input
GPS_Front    = clean_dataStructure.GPS_SparkFun_Front_ENU;
GPS_LeftRear = clean_dataStructure.GPS_SparkFun_LeftRear_ENU;
GPS_RightRear= clean_dataStructure.GPS_SparkFun_RightRear_ENU;

if any([isempty(GPS_Front), isempty(GPS_LeftRear), isempty(GPS_RightRear)]) || ...
   any([all(isnan(GPS_Front)), all(isnan(GPS_LeftRear)), all(isnan(GPS_RightRear))])
    error('Invalid GPS data.');
end

%% Parse inputs
Target_Center_ENU = targets_parameters(:,1:3);
Target_Diameters  = targets_parameters(:,4);
Target_Radii      = Target_Diameters / 2;
N_targets         = size(targets_parameters,1);
outliers_ratios    = 0.5*ones(N_targets,1);

%% Prepare outputs
fittedCenters_LiDAR = [];
measuredCenters_GPS = [];
fittedRadii_LiDAR   = [];
measuredRadii       = [];
fittedError = [];

%% Fit spheres
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _       
%  |  \/  |     (_)      
%  | \  / | __ _ _ _ __  
%  | |\/| |/ _` | | '_ \ 
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Loop through scans
LiDAR_pointcloud_cell = clean_dataStructure.LiDAR_Velodyne_Rear.PointCloud;
N_scans = length(LiDAR_pointcloud_cell);

for idx_scan = 1:N_scans
    
    point_cloud = LiDAR_pointcloud_cell{idx_scan};
    if isempty(point_cloud)
        continue;
    end

    % Compute GPS transformation for this scan
    M_RearRightGPS_to_ENU = fcn_Transform_CalculateTransformation_RearRightGPSToENU(...
        GPS_Front(idx_scan,:), GPS_LeftRear(idx_scan,:), GPS_RightRear(idx_scan,:));
    M_ENU_to_RearRightGPS = inv(M_RearRightGPS_to_ENU);
    M_RearRightGPS_to_LiDAR = inv(M_transform_LiDAR_to_GPS_previous);

    % Transform targets to LiDAR frame
    Target_Center_GPS   = se3(M_ENU_to_RearRightGPS).transform(Target_Center_ENU);
    Target_Center_LiDAR = se3(M_RearRightGPS_to_LiDAR).transform(Target_Center_GPS);

    for idx_target = 1:N_targets
        outliers_ratio = outliers_ratios(idx_target,1);
        target_center = Target_Center_LiDAR(idx_target,:);
        target_radius = Target_Radii(idx_target);
        radius_range = [target_radius - LiDAR_accuracy, target_radius + LiDAR_accuracy];
        scale = 1.5;
        constraint = scale * target_radius;
        x_range = [target_center(1)-constraint, target_center(1)+constraint];
        y_range = [target_center(2)-constraint, target_center(2)+constraint];
        z_range = [target_center(3)-constraint, target_center(3)+constraint];

        pc_roi = fcn_LiDARPoseEstimation_filterPointInXYZ(point_cloud, x_range, y_range, z_range);
        N_points = size(pc_roi,1);
        if size(pc_roi,1) < N_samples
            continue;
        end
        inliers_ratio = 1 - outliers_ratio;
        N_iter = round(log(1-0.99)/log(1-inliers_ratio^N_samples));
        N_iter = max([10 N_iter]); % Ensure lower bound
        [tf_inliers, updated_outliers_ratio] = fcn_LiDARPoseEstimation_identifyInliersWithRANSAC(...
            pc_roi, N_samples, N_iter, radius_range);
        outliers_ratios(idx_target,1) = updated_outliers_ratio;
        inlier_pts = pc_roi(tf_inliers,:);
        if size(inlier_pts,1) < N_points/2
            continue;
        end

        [C_fit, R_fit, residualErrors] = fcn_LiDARPoseEstimation_FitSphere_Algebraic(inlier_pts(:,1:3));

        residualError_ave = mean(residualErrors,'omitnan');
        % Use may change residual_thresh to discard points with larger
        % residual errors
        % residual_thresh = LiDAR_accuracy/2;
        residual_thresh = 0.01;
        if residualError_ave > residual_thresh
            continue;
        end
        if R_fit < radius_range(1) || R_fit > radius_range(2)
            continue;
        end

        %% Optional plotting for debugging
        if flag_do_plot
            figure(fig_num); 
            clf;
            scatter3(point_cloud(:,1), point_cloud(:,2), point_cloud(:,3), 10, 'k','filled');
            hold on
            scatter3(inlier_pts(:,1), inlier_pts(:,2), inlier_pts(:,3), 10, 'r','filled');
            scatter3(target_center(:,1), target_center(:,2), target_center(:,3), 100, 'm','*','LineWidth',3);
            scatter3(C_fit(:,1), C_fit(:,2), C_fit(:,3), 100, 'g','x','LineWidth',3);
       
            legend('All points','Inliers','Measured Center','Fitted center','location','best');
            axis equal; grid on;
            axis([x_range y_range z_range])
            xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
            title('RANSAC Sphere Fitting Result');
            pause(1)
        end
        
        % Accumulate output
        fittedCenters_LiDAR = [fittedCenters_LiDAR; C_fit];
        measuredCenters_GPS = [measuredCenters_GPS; Target_Center_GPS(idx_target,:)];
        fittedRadii_LiDAR   = [fittedRadii_LiDAR; R_fit];
        measuredRadii       = [measuredRadii; target_radius];
        fittedError = [fittedError; residualError_ave];
    end
end
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____       _
%  |  __ \     | |
%  | |  | | ___| |__  _   _  __ _
%  | |  | |/ _ \ '_ \| | | |/ _` |
%  | |__| |  __/ |_) | |_| | (_| |
%  |_____/ \___|_.__/ \__,_|\__, |
%                            __/ |
%                           |___/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag_do_debug
    fprintf('[DEBUG] Function %s completed.\n', mfilename);
end

end
