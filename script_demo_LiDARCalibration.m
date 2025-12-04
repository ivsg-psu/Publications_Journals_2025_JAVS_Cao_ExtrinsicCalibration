%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Introduction to and Purpose of the Code
% This is the demonstration script for the LiDAR extrinsic calibration
% pipeline using spherical targets and static scans.
%
% Script: Sphere Target Fitting and LiDAR Calibration
% Description:
%   - Fit multiple spherical targets from static LiDAR datasets
%   - Visualize fitted results and residual errors
%   - Register the LiDAR frame to GPS using sphere correspondences
%   - Apply the resulting transformation to lane marker center points
%   - Validate the transformation through geometric and angular residuals
%
% Author: Xinyu Cao
% Revised: 2025-07-21
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Step 0: Prepare the workspace
% -------------------------------------------------------------------------
% Purpose:
%   Clear the command window and close all existing figures to ensure
%   a clean environment before executing the calibration pipeline.
%
% Inputs:
%   None
% -------------------------------------------------------------------------
clc             % Clear the console
close all       % Close all open figures

%% Step 0.1: Initialize required dependencies
% -------------------------------------------------------------------------
% Purpose:
%   Define external library names, subfolders, and GitHub URLs.
%   These libraries will be downloaded and added to the path if needed.
%
% Inputs:
%   - DebugTools library (plotting/debugging tools)
%   - GeometryClass library (geometric utilities and data structures)
% -------------------------------------------------------------------------
clear library_name library_folders library_url

ith_library = 1;
library_name{ith_library}    = 'DebugTools_v2024_10_17';
library_folders{ith_library} = {'Functions','Data'};
library_url{ith_library}     = 'https://github.com/ivsg-psu/Errata_Tutorials_DebugTools/archive/refs/tags/DebugTools_v2024_10_17.zip';

ith_library = ith_library+1;
library_name{ith_library}    = 'GeometryClass_v2024_08_28';
library_folders{ith_library} = {'Functions','Data'};
library_url{ith_library}     = 'https://github.com/ivsg-psu/PathPlanning_GeomTools_GeomClassLibrary/archive/refs/tags/GeometryClass_v2024_08_28.zip';

%% Step 0.2: Clear cached paths if needed
% -------------------------------------------------------------------------
% Purpose:
%   Optional debug step. Clears all cached folders and resets utilities
%   in case previous paths interfere with current execution.
%
% Inputs:
%   Set manually inside this section.
% -------------------------------------------------------------------------
if 1==0
    clear flag_GeomClass_Folders_Initialized;
    fcn_INTERNAL_clearUtilitiesFromPathAndFolders;
end

%% Step 0.3: Initialize project folders and add dependencies to path
% -------------------------------------------------------------------------
% Purpose:
%   Add required project and external library folders to MATLAB path.
%   Skip this step if already initialized.
%
% Inputs:
%   Project-specific folders: 'Functions', 'Data', 'LargeData'
% -------------------------------------------------------------------------
if ~exist('flag_GeomClass_Folders_Initialized','var')
    this_project_folders = {'Functions','Data','LargeData'};
    fcn_INTERNAL_initializeUtilities(library_name,library_folders,library_url,this_project_folders);
    flag_GeomClass_Folders_Initialized = 1;
end

%% Step 1: Load static LiDAR scans with sphere targets
% -------------------------------------------------------------------------
% Purpose:
%   Load static point cloud data that captures spherical targets.
%   This data will be used for fitting spheres and computing calibration.
%
% Inputs:
%   - calibration_datasets: contains static point cloud scans
%   - targets_parameters.mat: includes ground truth info about target spheres
%   - GPS/vehicle transformation info for later validation
% -------------------------------------------------------------------------
folder_path = 'Data/Calibration';
calibration_datasets = fcn_LiDARPoseEstimation_loadDatasets(folder_path);
load('Data/Transformation/M_calibration_GPS_to_Vehicle.mat','M_calibration_GPS_to_Vehicle');
load('Data/Transformation/RearRightGPS_offset_relative_to_VehicleOrigin.mat','RearRightGPS_offset_relative_to_VehicleOrigin');
load('Data/Transformation/M_transform_LiDARVelodyne_to_RearRightGPS.mat','M_transform_LiDARVelodyne_to_RearRightGPS');
ref_baseStationLLA = [40.86368573 -77.83592832 344.189];  % PSU test track
load('Data/targets_parameters.mat','targets_parameters');

%% Step 2: Load or define initial calibration between LiDAR and GPS
% -------------------------------------------------------------------------
% Purpose:
%   Load a previously saved transformation matrix or define an initial
%   manual estimate of the LiDAR-to-GPS transformation. This will be used
%   for projecting ENU targets into the LiDAR frame in subsequent steps.
%
% Inputs:
%   - Previously saved transformation matrix (optional)
%   - Manual offsets and angles if no calibration file is used
%
% NOTE:
% Manually measured initial transformation may be inaccurate and result in
% incorrect filtering of target regions. It is recommended to use the
% initial calibration only once to get a coarse alignment, and then repeat
% the process with the updated result to obtain improved accuracy.
% -------------------------------------------------------------------------
flag_load_previous_calibration = 0;
if flag_load_previous_calibration == 1
   
    load('Data/Transformation/M_transform_LiDARVelodyne_to_RearRightGPS.mat','M_transform_LiDARVelodyne_to_RearRightGPS');
    M_transform_LiDARVelodyne_to_RearRightGPS_Prev = ...
        se3(M_transform_LiDARVelodyne_to_RearRightGPS);
else
    antenna_center_height = (92.075+43.60)/1000;
    dx = -0.7903;
    dy = 0.7295 + 0.056;
    dz = 0.1457 - antenna_center_height;
    LiDARVelodyne_offset_relative_to_RearRightGPS = [dx, dy, dz];
    x_angle = 0;
    y_angle = deg2rad(36);
    z_angle = deg2rad(180);
    M_transform_LiDARVelodyne_to_RearRightGPS_Prev = ...
        fcn_Transform_CreateTransformationMatrix( ...
            LiDARVelodyne_offset_relative_to_RearRightGPS, ...
            x_angle, y_angle, z_angle);
end

%% Step 2.1: Visualize known targets projected into LiDAR frame
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
sample_data = calibration_datasets{1};
idx_scan_to_visualize = 12;
fig_num = 21;
fcn_LiDARPoseEstimation_viewSphereTargets( ...
    sample_data, ...
    M_transform_LiDARVelodyne_to_RearRightGPS_Prev, ...
    targets_parameters, ...
    idx_scan_to_visualize, fig_num);

%% Step 2.2: Visualize constraint space for a single target
% -------------------------------------------------------------------------
% Purpose:
%   Visualize the constraint space (search region) for one selected target
%   before applying RANSAC fitting. Useful for debugging and confirmation.
%
% Inputs:
%   - Index of dataset and scan to visualize
%   - Target index to inspect
% -------------------------------------------------------------------------
dataset_to_view = 1;
sample_data = calibration_datasets{dataset_to_view};
idx_scan_to_visualize = 1;
idx_target_to_visualize = 2;
fig_num = 22;
fcn_LiDARPoseEstimation_visualizeConstraintSpace( ...
    sample_data, ...
    M_transform_LiDARVelodyne_to_RearRightGPS_Prev, ...
    targets_parameters, ...
    idx_scan_to_visualize, ...
    idx_target_to_visualize, fig_num, [], 1);

%% Step 3: Fit spheres from LiDAR scans using RANSAC + LSQ
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
N_samples = 4;
LiDAR_accuracy = 0.03;
dist_thresh_input = LiDAR_accuracy;

tic
N_dataset = length(calibration_datasets);
fittedCenters_all_LiDAR = [];
measuredCenters_all_GPS = [];
fittedRadii_all_LiDAR = [];
measuredRadii_all = [];
fittedError_all = [];
% Change fig_num to 30 or other integer to plot for debugging
fig_num = -1;

for ith_dataset = 1:N_dataset
    example_dataset_ith = calibration_datasets{ith_dataset};
    if isempty(example_dataset_ith)
        continue;
    end
    [fittedCenters_LiDAR, measuredCenters_GPS, ...
        fittedRadii_LiDAR, measuredRadii, fittedError] = ...
        fcn_LiDARPoseEstimation_FitSphereTarget( ...
            example_dataset_ith, ...
            targets_parameters, ...
            M_transform_LiDARVelodyne_to_RearRightGPS, ...
            N_samples, LiDAR_accuracy, fig_num);
    fittedCenters_all_LiDAR = [fittedCenters_all_LiDAR; fittedCenters_LiDAR];
    measuredCenters_all_GPS = [measuredCenters_all_GPS; measuredCenters_GPS];
    fittedRadii_all_LiDAR   = [fittedRadii_all_LiDAR; fittedRadii_LiDAR];
    measuredRadii_all       = [measuredRadii_all; measuredRadii];
    fittedError_all         = [fittedError_all; fittedError];
end
toc

%% Step 3.1: Visualize sphere fitting residuals by radius
% -------------------------------------------------------------------------
% Purpose:
%   Plot residual errors for fitted spheres grouped by radius.
%   Helps identify systematic bias or noise across different sizes.
%
% Inputs:
%   - Fitted and measured radii
%   - Residual fitting errors from Step 3
% -------------------------------------------------------------------------
fig_num = 31;
fcn_LiDARPoseEstimation_visualizeSphereFitResidual( ...
    targets_parameters, ...
    measuredRadii_all, ...
    fittedError_all, ...
    fig_num);

%% Step 4: Estimate transformation using sphere correspondences
% -------------------------------------------------------------------------
% Purpose:
%   Refine the LiDAR-to-GPS transformation by minimizing alignment error
%   between fitted and measured sphere centers using iterative registration.
%
% Inputs:
%   - Sphere centers in LiDAR frame and GPS frame
%   - Fitted and measured radii (used as weights)
% -------------------------------------------------------------------------
max_iterations = 100;
sourcePoints = fittedCenters_all_LiDAR;
targetPoints = measuredCenters_all_GPS;
sphereRadius = fittedRadii_all_LiDAR;
referenceRadius = measuredRadii_all;

M_transform_LiDARVelodyne_to_RearRightGPS = ...
    fcn_LiDARPoseEstimation_iterativePointRegistration( ...
        sourcePoints, targetPoints, ...
        sphereRadius, referenceRadius, ...
        max_iterations);

%% Step 4.1: Visualize aligned spheres in ENU frame
% -------------------------------------------------------------------------
% Purpose:
%   Display the fitted and transformed spheres in ENU coordinates to
%   confirm visually that registration worked as expected.
%
% Inputs:
%   - Sample scan
%   - Final transformation matrix
%   - Target geometry parameters
% -------------------------------------------------------------------------
dataset_to_view = 3;
sample_data = calibration_datasets{dataset_to_view};
idx_scan_to_visualize = 2;
fig_num = 41;
fcn_LiDARPoseEstimation_viewFittedSphere( ...
    sample_data, ...
    M_transform_LiDARVelodyne_to_RearRightGPS, ...
    targets_parameters, ...
    idx_scan_to_visualize, fig_num, 'ENU');

%% Step 5: Load lane marker datasets for transformation validation
% -------------------------------------------------------------------------
% Purpose:
%   Load pre-extracted lane marker center points and LiDAR point clouds
%   for use in validating the LiDAR-to-GPS calibration results.
%
% Inputs:
%   - LaneMarkerDataset_Cell.mat
%   - XYZ_LaneMarker_Centers_Cell.mat
% -------------------------------------------------------------------------
test_folder_path = 'Data/Test';
LaneMarkerDataset_Cell = fcn_LiDARPoseEstimation_loadDatasets(test_folder_path, 'test_dataset');
load('Data/XYZ_LaneMarker_Centers_Cell.mat','XYZ_LaneMarker_Centers_Cell');
load('Data/LaneMarker_HandMeasurement_ENU.mat','LaneMarker_HandMeasurement_ENU')
%% Step 5.1: Visualize lane markers after transformation
% -------------------------------------------------------------------------
% Purpose:
%   Project and compare extracted lane marker center points against
%   hand-labeled ENU ground truth for a single scan.
%
% Inputs:
%   - Single scan from lane marker dataset
%   - Transformation matrix
% -------------------------------------------------------------------------
dataset_to_view = 1;
scan_to_view = 1;
LaneMarker_dataset = LaneMarkerDataset_Cell{dataset_to_view};
LaneMarker_Centers_inLiDAR = XYZ_LaneMarker_Centers_Cell{dataset_to_view};
fig_num = 51;
fcn_LiDARPoseEstimation_plotLaneMarkersENU( ...
    LaneMarker_dataset, ...
    LaneMarker_HandMeasurement_ENU, ...
    scan_to_view, ...
    M_transform_LiDARVelodyne_to_RearRightGPS, fig_num);

%% Step 5.2: Apply transformation to all lane marker points
% -------------------------------------------------------------------------
% Purpose:
%   Transform all lane marker center points and LiDAR point clouds
%   in batch to the ENU frame for quantitative validation.
%
% Inputs:
%   - All lane marker datasets and center point lists
%   - Final estimated transformation matrix
% -------------------------------------------------------------------------
N_datasets = length(LaneMarkerDataset_Cell);
lane_marker_points_ENU_cell = cell(N_datasets,1);
LaneMarkerENU_dataset = cell(N_datasets,1);

for idx_dataset = 1:N_datasets
    LaneMarker_dataset = LaneMarkerDataset_Cell{idx_dataset};
    LiDAR_PointCloud = LaneMarker_dataset.LiDAR_Velodyne_Rear.PointCloud;
    LaneMarker_Centers_inLiDAR = XYZ_LaneMarker_Centers_Cell{idx_dataset};

    lane_marker_points_ENU = fcn_LiDARPoseEstimation_transformLiDARToENU( ...
        LaneMarker_dataset, LaneMarker_Centers_inLiDAR, ...
        M_transform_LiDARVelodyne_to_RearRightGPS);
    LiDAR_PointCloud_ENU = fcn_LiDARPoseEstimation_transformLiDARToENU( ...
        LaneMarker_dataset, LiDAR_PointCloud, ...
        M_transform_LiDARVelodyne_to_RearRightGPS);

    lane_marker_points_ENU_cell{idx_dataset} = cell2mat(lane_marker_points_ENU);
    test_dataset_ENU = LaneMarker_dataset;
    test_dataset_ENU.LiDAR_Velodyne_Rear.PointCloud = LiDAR_PointCloud_ENU;
    LaneMarkerENU_dataset{idx_dataset} = test_dataset_ENU;
end

%% Step 5.3: Compare road surface pitch/roll before and after transformation
% -------------------------------------------------------------------------
% Purpose:
%   Compare estimated vehicle pose orientation to ground plane
%   to evaluate if transformation preserves pitch/roll alignment.
%
% Inputs:
%   - ENU-transformed dataset
%   - GPS-to-vehicle reference transformation
% -------------------------------------------------------------------------
ave_roll_offset = [];
ave_pitch_offset = [];
for idx_dataset = 1:N_datasets
    test_dataset_ENU = LaneMarkerENU_dataset{idx_dataset};
    [roll_offset, pitch_offset] = fcn_LiDARPoseEstimation_compareGroundAngle( ...
        test_dataset_ENU, ...
        LaneMarker_HandMeasurement_ENU);
    ave_roll_offset = [ave_roll_offset; mean(roll_offset)];
    ave_pitch_offset = [ave_pitch_offset; mean(pitch_offset)];
end
% Compute statistics
mean_roll = mean(ave_roll_offset, 'omitnan');
std_roll  = std(ave_roll_offset,  'omitnan');

mean_pitch = mean(ave_pitch_offset, 'omitnan');
std_pitch  = std(ave_pitch_offset,  'omitnan');

% Display
fprintf('Roll Offset:  Mean = %.3f deg, Std = %.3f deg\n', mean_roll, std_roll);
fprintf('Pitch Offset: Mean = %.3f deg, Std = %.3f deg\n', mean_pitch, std_pitch);
%% Step 5.4: Quantitatively evaluate registration residuals
% -------------------------------------------------------------------------
% Purpose:
%   Compute Euclidean residuals between transformed lane marker centers
%   and manually labeled ground truth to quantify calibration accuracy.
%
% Inputs:
%   - Hand-measured ENU reference points (ground truth)
%   - Transformed lane marker center points
% -------------------------------------------------------------------------
fig_num = 54;
sigma_scale = 2;
fcn_LiDARPoseEstimation_compareMatchedPoints( ...
    LaneMarker_HandMeasurement_ENU, ...
    lane_marker_points_ENU_cell, ...
    fig_num,2);

%% Functions follow
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   ______                _   _
%  |  ____|              | | (_)
%  | |__ _   _ _ __   ___| |_ _  ___  _ __  ___
%  |  __| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
%  | |  | |_| | | | | (__| |_| | (_) | | | \__ \
%  |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
%
% See: https://patorjk.com/software/taag/#p=display&f=Big&t=Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%§

%%
function only_fitted_sphere_points = fcn_INTERNAL_plotResults(time_iteration, mean_circle_center, RingNames, currentScanRaw, rings_in_agreement, good_Regression_domains_to_keep, good_axis_limits)


title(sprintf('Time: %.0d',time_iteration));
N_rings = length(RingNames);
if ~isempty(mean_circle_center)
    figure(47464);
    clf;
    hold on;
    grid on;
    axis equal
    plot3(mean_circle_center(:,1), mean_circle_center(:,2), mean_circle_center(:,3),'r.','MarkerSize',50, 'Linewidth',3);
    % Get the color ordering?
    try
        color_ordering = orderedcolors('gem12');
    catch
        color_ordering = colororder;
    end

    N_colors = length(color_ordering(:,1));

    % Start by plotting the entire scan

    for ith_ring = 1:N_rings
        ringName = RingNames{ith_ring};
        RingDataRaw = currentScanRaw.(ringName);
        plot3(RingDataRaw(:,1), RingDataRaw(:,2), RingDataRaw(:,3),'k.','MarkerSize',10);
    end
    view(-15,55)
    axis(good_axis_limits);



end



% Initialize arrays
all_close_RingData = {};
all_fitted_RingData = {};
only_fitted_sphere_points = [];

if 1==0
    % Pull out and plot all the fits
    for ith_ring = 1:length(rings_in_agreement)
        current_ring = rings_in_agreement(ith_ring);
        ringName = RingNames{current_ring};
        RingDataRaw = currentScanRaw.(ringName);
        pointData3d = RingDataRaw(:,1:3);

        % Find the fitted data
        % pointData2d = good_Hough_domains_to_keep{ith_ring}.points_in_domain;
        pointData2d = good_Regression_domains_to_keep{ith_ring}.points_in_domain;
        fitted_RingData = [];
        for ith_point = 1:length(pointData2d(:,1))
            % Find where the points match to 5 decimal places
            N_decimals = 4;
            point2d = floor(pointData2d(ith_point,1:2)*(10^N_decimals));
            points3d = floor(pointData3d(:,1:2)*(10^N_decimals));
            [flag_match_find, index_of_match] = ismember(point2d, points3d, 'rows');
            if ~flag_match_find
                error('no match found');
            end

            fitted_RingData = [fitted_RingData; RingDataRaw(index_of_match,:)]; %#ok<AGROW>
        end

        % Save the results
        all_fitted_RingData{ith_ring} = fitted_RingData; %#ok<AGROW>


        % Plot results
        current_color = color_ordering(mod(ith_ring,N_colors)+1,:);
        plot3(all_fitted_RingData{ith_ring}(:,1), all_fitted_RingData{ith_ring}(:,2), all_fitted_RingData{ith_ring}(:,3),'.','MarkerSize',30,'Color',current_color);
    end
end

% Pull out and plot all the close points in any ring.
% Set a distance threshold
threshold_point_distance = 0.35; % Actual radius is about 0.336
for ith_ring = 1:N_rings
    ringName = RingNames{ith_ring};
    RingDataRaw = currentScanRaw.(ringName);
    pointData3d = RingDataRaw(:,1:3);

    % Find the close data
    if ~isempty(pointData3d)&~isempty(mean_circle_center)
        distance_to_mean_center = sum((pointData3d(:,1:3)-mean_circle_center(1,1:3)).^2,2).^0.5;
        close_data = RingDataRaw(distance_to_mean_center<threshold_point_distance,:);

    % Save the results
        all_close_RingData{ith_ring} = close_data; %#ok<AGROW>
        only_fitted_sphere_points = [only_fitted_sphere_points; close_data]; %#ok<AGROW>

    % Plot results
        plot3(all_close_RingData{ith_ring}(:,1), all_close_RingData{ith_ring}(:,2), all_close_RingData{ith_ring}(:,3),'.','MarkerSize',15,'Color',[1 0 0]);
    end
end
    if ~isempty(only_fitted_sphere_points)
        plot3(only_fitted_sphere_points(:,1), only_fitted_sphere_points(:,2), only_fitted_sphere_points(:,3),'.','MarkerSize',10,'Color',[1  1 0]);
    end
end

%% function fcn_INTERNAL_clearUtilitiesFromPathAndFolders
function fcn_INTERNAL_clearUtilitiesFromPathAndFolders
% Clear out the variables
clear global flag* FLAG*
clear flag*
clear path

% Clear out any path directories under Utilities
path_dirs = regexp(path,'[;]','split');
utilities_dir = fullfile(pwd,filesep,'Utilities');
for ith_dir = 1:length(path_dirs)
    utility_flag = strfind(path_dirs{ith_dir},utilities_dir);
    if ~isempty(utility_flag)
        rmpath(path_dirs{ith_dir});
    end
end

% Delete the Utilities folder, to be extra clean!
if  exist(utilities_dir,'dir')
    [status,message,message_ID] = rmdir(utilities_dir,'s');
    if 0==status
        error('Unable remove directory: %s \nReason message: %s \nand message_ID: %s\n',utilities_dir, message,message_ID);
    end
end

end % Ends fcn_INTERNAL_clearUtilitiesFromPathAndFolders

%% fcn_INTERNAL_initializeUtilities
function  fcn_INTERNAL_initializeUtilities(library_name,library_folders,library_url,this_project_folders)
% Reset all flags for installs to empty
clear global FLAG*

fprintf(1,'Installing utilities necessary for code ...\n');

% Dependencies and Setup of the Code
% This code depends on several other libraries of codes that contain
% commonly used functions. We check to see if these libraries are installed
% into our "Utilities" folder, and if not, we install them and then set a
% flag to not install them again.

% Set up libraries
for ith_library = 1:length(library_name)
    dependency_name = library_name{ith_library};
    dependency_subfolders = library_folders{ith_library};
    dependency_url = library_url{ith_library};
    
    fprintf(1,'\tAdding library: %s ...',dependency_name);
    fcn_INTERNAL_DebugTools_installDependencies(dependency_name, dependency_subfolders, dependency_url);
    clear dependency_name dependency_subfolders dependency_url
    fprintf(1,'Done.\n');
end

% Set dependencies for this project specifically
fcn_DebugTools_addSubdirectoriesToPath(pwd,this_project_folders);

disp('Done setting up libraries, adding each to MATLAB path, and adding current repo folders to path.');
end % Ends fcn_INTERNAL_initializeUtilities


function fcn_INTERNAL_DebugTools_installDependencies(dependency_name, dependency_subfolders, dependency_url, varargin)
%% FCN_DEBUGTOOLS_INSTALLDEPENDENCIES - MATLAB package installer from URL
%
% FCN_DEBUGTOOLS_INSTALLDEPENDENCIES installs code packages that are
% specified by a URL pointing to a zip file into a default local subfolder,
% "Utilities", under the root folder. It also adds either the package
% subfoder or any specified sub-subfolders to the MATLAB path.
%
% If the Utilities folder does not exist, it is created.
%
% If the specified code package folder and all subfolders already exist,
% the package is not installed. Otherwise, the folders are created as
% needed, and the package is installed.
%
% If one does not wish to put these codes in different directories, the
% function can be easily modified with strings specifying the
% desired install location.
%
% For path creation, if the "DebugTools" package is being installed, the
% code installs the package, then shifts temporarily into the package to
% complete the path definitions for MATLAB. If the DebugTools is not
% already installed, an error is thrown as these tools are needed for the
% path creation.
%
% Finally, the code sets a global flag to indicate that the folders are
% initialized so that, in this session, if the code is called again the
% folders will not be installed. This global flag can be overwritten by an
% optional flag input.
%
% FORMAT:
%
%      fcn_DebugTools_installDependencies(...
%           dependency_name, ...
%           dependency_subfolders, ...
%           dependency_url)
%
% INPUTS:
%
%      dependency_name: the name given to the subfolder in the Utilities
%      directory for the package install
%
%      dependency_subfolders: in addition to the package subfoder, a list
%      of any specified sub-subfolders to the MATLAB path. Leave blank to
%      add only the package subfolder to the path. See the example below.
%
%      dependency_url: the URL pointing to the code package.
%
%      (OPTIONAL INPUTS)
%      flag_force_creation: if any value other than zero, forces the
%      install to occur even if the global flag is set.
%
% OUTPUTS:
%
%      (none)
%
% DEPENDENCIES:
%
%      This code will automatically get dependent files from the internet,
%      but of course this requires an internet connection. If the
%      DebugTools are being installed, it does not require any other
%      functions. But for other packages, it uses the following from the
%      DebugTools library: fcn_DebugTools_addSubdirectoriesToPath
%
% EXAMPLES:
%
% % Define the name of subfolder to be created in "Utilities" subfolder
% dependency_name = 'DebugTools_v2023_01_18';
%
% % Define sub-subfolders that are in the code package that also need to be
% % added to the MATLAB path after install; the package install subfolder
% % is NOT added to path. OR: Leave empty ({}) to only add
% % the subfolder path without any sub-subfolder path additions.
% dependency_subfolders = {'Functions','Data'};
%
% % Define a universal resource locator (URL) pointing to the zip file to
% % install. For example, here is the zip file location to the Debugtools
% % package on GitHub:
% dependency_url = 'https://github.com/ivsg-psu/Errata_Tutorials_DebugTools/blob/main/Releases/DebugTools_v2023_01_18.zip?raw=true';
%
% % Call the function to do the install
% fcn_DebugTools_installDependencies(dependency_name, dependency_subfolders, dependency_url)
%
% This function was written on 2023_01_23 by S. Brennan
% Questions or comments? sbrennan@psu.edu

% Revision history:
% 2023_01_23:
% -- wrote the code originally
% 2023_04_20:
% -- improved error handling
% -- fixes nested installs automatically

% TO DO
% -- Add input argument checking

flag_do_debug = 0; % Flag to show the results for debugging
flag_do_plots = 0; % % Flag to plot the final results
flag_check_inputs = 1; % Flag to perform input checking

if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
end


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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if flag_check_inputs
    % Are there the right number of inputs?
    narginchk(3,4);
end

%% Set the global variable - need this for input checking
% Create a variable name for our flag. Stylistically, global variables are
% usually all caps.
flag_varname = upper(cat(2,'flag_',dependency_name,'_Folders_Initialized'));

% Make the variable global
eval(sprintf('global %s',flag_varname));

if nargin==4
    if varargin{1}
        eval(sprintf('clear global %s',flag_varname));
    end
end

%% Main code starts here
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _
%  |  \/  |     (_)
%  | \  / | __ _ _ _ __
%  | |\/| |/ _` | | '_ \
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



if ~exist(flag_varname,'var') || isempty(eval(flag_varname))
    % Save the root directory, so we can get back to it after some of the
    % operations below. We use the Print Working Directory command (pwd) to
    % do this. Note: this command is from Unix/Linux world, but is so
    % useful that MATLAB made their own!
    root_directory_name = pwd;
    
    % Does the directory "Utilities" exist?
    utilities_folder_name = fullfile(root_directory_name,'Utilities');
    if ~exist(utilities_folder_name,'dir')
        % If we are in here, the directory does not exist. So create it
        % using mkdir
        [success_flag,error_message,message_ID] = mkdir(root_directory_name,'Utilities');
        
        % Did it work?
        if ~success_flag
            error('Unable to make the Utilities directory. Reason: %s with message ID: %s\n',error_message,message_ID);
        elseif ~isempty(error_message)
            warning('The Utilities directory was created, but with a warning: %s\n and message ID: %s\n(continuing)\n',error_message, message_ID);
        end
        
    end
    
    % Does the directory for the dependency folder exist?
    dependency_folder_name = fullfile(root_directory_name,'Utilities',dependency_name);
    if ~exist(dependency_folder_name,'dir')
        % If we are in here, the directory does not exist. So create it
        % using mkdir
        [success_flag,error_message,message_ID] = mkdir(utilities_folder_name,dependency_name);
        
        % Did it work?
        if ~success_flag
            error('Unable to make the dependency directory: %s. Reason: %s with message ID: %s\n',dependency_name, error_message,message_ID);
        elseif ~isempty(error_message)
            warning('The %s directory was created, but with a warning: %s\n and message ID: %s\n(continuing)\n',dependency_name, error_message, message_ID);
        end
        
    end
    
    % Do the subfolders exist?
    flag_allFoldersThere = 1;
    if isempty(dependency_subfolders{1})
        flag_allFoldersThere = 0;
    else
        for ith_folder = 1:length(dependency_subfolders)
            subfolder_name = dependency_subfolders{ith_folder};
            
            % Create the entire path
            subfunction_folder = fullfile(root_directory_name, 'Utilities', dependency_name,subfolder_name);
            
            % Check if the folder and file exists that is typically created when
            % unzipping.
            if ~exist(subfunction_folder,'dir')
                flag_allFoldersThere = 0;
            end
        end
    end
    
    % Do we need to unzip the files?
    if flag_allFoldersThere==0
        % Files do not exist yet - try unzipping them.
        save_file_name = tempname(root_directory_name);
        zip_file_name = websave(save_file_name,dependency_url);
        % CANT GET THIS TO WORK --> unzip(zip_file_url, debugTools_folder_name);
        
        % Is the file there?
        if ~exist(zip_file_name,'file')
            error(['The zip file: %s for dependency: %s did not download correctly.\n' ...
                'This is usually because permissions are restricted on ' ...
                'the current directory. Check the code install ' ...
                '(see README.md) and try again.\n'],zip_file_name, dependency_name);
        end
        
        % Try unzipping
        unzip(zip_file_name, dependency_folder_name);
        
        % Did this work? If so, directory should not be empty
        directory_contents = dir(dependency_folder_name);
        if isempty(directory_contents)
            error(['The necessary dependency: %s has an error in install ' ...
                'where the zip file downloaded correctly, ' ...
                'but the unzip operation did not put any content ' ...
                'into the correct folder. ' ...
                'This suggests a bad zip file or permissions error ' ...
                'on the local computer.\n'],dependency_name);
        end
        
        % Check if is a nested install (for example, installing a folder
        % "Toolsets" under a folder called "Toolsets"). This can be found
        % if there's a folder whose name contains the dependency_name
        flag_is_nested_install = 0;
        for ith_entry = 1:length(directory_contents)
            if contains(directory_contents(ith_entry).name,dependency_name)
                if directory_contents(ith_entry).isdir
                    flag_is_nested_install = 1;
                    install_directory_from = fullfile(directory_contents(ith_entry).folder,directory_contents(ith_entry).name);
                    install_files_from = fullfile(directory_contents(ith_entry).folder,directory_contents(ith_entry).name,'*'); % BUG FIX - For Macs, must be *, not *.*
                    install_location_to = fullfile(directory_contents(ith_entry).folder);
                end
            end
        end
        
        if flag_is_nested_install
            [status,message,message_ID] = movefile(install_files_from,install_location_to);
            if 0==status
                error(['Unable to move files from directory: %s\n ' ...
                    'To: %s \n' ...
                    'Reason message: %s\n' ...
                    'And message_ID: %s\n'],install_files_from,install_location_to, message,message_ID);
            end
            [status,message,message_ID] = rmdir(install_directory_from);
            if 0==status
                error(['Unable remove directory: %s \n' ...
                    'Reason message: %s \n' ...
                    'And message_ID: %s\n'],install_directory_from,message,message_ID);
            end
        end
        
        % Make sure the subfolders were created
        flag_allFoldersThere = 1;
        if ~isempty(dependency_subfolders{1})
            for ith_folder = 1:length(dependency_subfolders)
                subfolder_name = dependency_subfolders{ith_folder};
                
                % Create the entire path
                subfunction_folder = fullfile(root_directory_name, 'Utilities', dependency_name,subfolder_name);
                
                % Check if the folder and file exists that is typically created when
                % unzipping.
                if ~exist(subfunction_folder,'dir')
                    flag_allFoldersThere = 0;
                end
            end
        end
        % If any are not there, then throw an error
        if flag_allFoldersThere==0
            error(['The necessary dependency: %s has an error in install, ' ...
                'or error performing an unzip operation. The subfolders ' ...
                'requested by the code were not found after the unzip ' ...
                'operation. This suggests a bad zip file, or a permissions ' ...
                'error on the local computer, or that folders are ' ...
                'specified that are not present on the remote code ' ...
                'repository.\n'],dependency_name);
        else
            % Clean up the zip file
            delete(zip_file_name);
        end
        
    end
    
    
    % For path creation, if the "DebugTools" package is being installed, the
    % code installs the package, then shifts temporarily into the package to
    % complete the path definitions for MATLAB. If the DebugTools is not
    % already installed, an error is thrown as these tools are needed for the
    % path creation.
    %
    % In other words: DebugTools is a special case because folders not
    % added yet, and we use DebugTools for adding the other directories
    if strcmp(dependency_name(1:10),'DebugTools')
        debugTools_function_folder = fullfile(root_directory_name, 'Utilities', dependency_name,'Functions');
        
        % Move into the folder, run the function, and move back
        cd(debugTools_function_folder);
        fcn_DebugTools_addSubdirectoriesToPath(dependency_folder_name,dependency_subfolders);
        cd(root_directory_name);
    else
        try
            fcn_DebugTools_addSubdirectoriesToPath(dependency_folder_name,dependency_subfolders);
        catch
            error(['Package installer requires DebugTools package to be ' ...
                'installed first. Please install that before ' ...
                'installing this package']);
        end
    end
    
    
    % Finally, the code sets a global flag to indicate that the folders are
    % initialized.  Check this using a command "exist", which takes a
    % character string (the name inside the '' marks, and a type string -
    % in this case 'var') and checks if a variable ('var') exists in matlab
    % that has the same name as the string. The ~ in front of exist says to
    % do the opposite. So the following command basically means: if the
    % variable named 'flag_CodeX_Folders_Initialized' does NOT exist in the
    % workspace, run the code in the if statement. If we look at the bottom
    % of the if statement, we fill in that variable. That way, the next
    % time the code is run - assuming the if statement ran to the end -
    % this section of code will NOT be run twice.
    
    eval(sprintf('%s = 1;',flag_varname));
end


%% Plot the results (for debugging)?
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
if flag_do_plots
    
    % Nothing to do!
    
    
    
end

if flag_do_debug
    fprintf(1,'ENDING function: %s, in file: %s\n\n',st(1).name,st(1).file);
end

end % Ends function fcn_DebugTools_installDependencies


    