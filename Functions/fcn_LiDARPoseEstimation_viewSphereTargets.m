function fcn_LiDARPoseEstimation_viewSphereTargets(...
    dataset, T_LiDAR_to_RearRightGPS, targets_parameters, ...
    idx_scan_to_visualize, fig_num, varargin)
% fcn_LiDARVisualization_plotIdentifiedTargets
% Visualizes LiDAR scan with target regions in specified coordinate frame.
%
% INPUTS:
%   clean_data: struct with fields:
%       - LiDAR_Velodyne_Rear.PointCloud
%       - GPS_SparkFun_Front_ENU, *_LeftRear_ENU, *_RightRear_ENU
%   T_LiDAR_to_RearRightGPS: se3 object
%       Transform from LiDAR to rear-right GPS
%   targets_parameters: Mx4
%       Ground truth sphere centers in ENU and diamsters of the spheres
%   idx_scan_to_visualize: scalar
%       Index of scan to visualize
%   fig_num: scalar
%       Figure number to draw
%   view_frame: (optional) string
%       Coordinate frame to visualize in: 'LiDAR' (default), 'GPS', or 'ENU'
%

% This function was written on 2025_03_20 by X.Cao
% Questions or comments? xfc5113@psu.edu

% Revision history:
% 2025_03_20 - wrote the code
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
narginchk(5,7);

view_frame = 'LiDAR';
if nargin >= 6
    view_frame = varargin{1};
    if isempty(view_frame)
        view_frame = 'LiDAR';
    end
end
N_targets = size(targets_parameters, 1);
end_target_index = N_targets;
Target_Center_ENU = targets_parameters(:,1:3);
Target_Diameters = targets_parameters(:,4);
start_target_idx = 1;


if nargin >= 7
    start_target_idx = varargin{2}(1);
    end_target_index = varargin{2}(2);
end


%% Main function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _       
%  |  \/  |     (_)      
%  | \  / | __ _ _ _ __  
%  | |\/| |/ _` | | '_ \ 
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Extract scan and GPS data
ptCloud_cell = dataset.LiDAR_Velodyne_Rear.PointCloud;
ptCloud_current = ptCloud_cell{idx_scan_to_visualize};

GPS_Front = dataset.GPS_SparkFun_Front_ENU(idx_scan_to_visualize,:);
GPS_LeftRear = dataset.GPS_SparkFun_LeftRear_ENU(idx_scan_to_visualize,:);
GPS_RightRear = dataset.GPS_SparkFun_RightRear_ENU(idx_scan_to_visualize,:);

%% Compute transforms
M_GPS_to_ENU = fcn_Transform_CalculateTransformation_RearRightGPSToENU(...
    GPS_Front, GPS_LeftRear, GPS_RightRear);
T_GPS_to_ENU = se3(M_GPS_to_ENU);
T_LiDAR_to_ENU = T_GPS_to_ENU * T_LiDAR_to_RearRightGPS;
T_ENU_to_LiDAR = inv(T_LiDAR_to_ENU);
T_LiDAR_to_GPS = inv(T_LiDAR_to_RearRightGPS);
T_ENU_to_GPS = inv(T_GPS_to_ENU);

%% Transform according to view_frame
switch lower(view_frame)
    case 'lidar'
        ptCloud_transformed = ptCloud_current;
        Target_Center_transformed = T_ENU_to_LiDAR.transform(Target_Center_ENU);
    case 'gps'
        ptCloud_transformed = T_LiDAR_to_GPS.transform(ptCloud_current(:,1:3));
        ptCloud_transformed = [ptCloud_transformed, ptCloud_current(:,4:end)];
        Target_Center_transformed = T_ENU_to_GPS.transform(Target_Center_ENU);
    case 'enu'
        ptCloud_transformed = T_LiDAR_to_ENU.transform(ptCloud_current(:,1:3));
        ptCloud_transformed = [ptCloud_transformed, ptCloud_current(:,4:end)];
        Target_Center_transformed = Target_Center_ENU;
    otherwise
        error('Unsupported view_frame. Use ''lidar'', ''gps'', or ''enu''.');
end



%% Plotting
figure(fig_num); 
clf;
scatter3(ptCloud_transformed(:,1), ptCloud_transformed(:,2), ptCloud_transformed(:,3), 10, 'k','filled');
hold on;
xyz_limits = [inf -inf; inf -inf; inf -inf];  % [x_min x_max; y_min y_max; z_min z_max]


for idx_target = start_target_idx:end_target_index
    center = Target_Center_transformed(idx_target,:);
    radius = Target_Diameters(idx_target) / 2;

    x_rng = [center(1) - 1.5*radius, center(1) + 1.5*radius];
    y_rng = [center(2) - 1.5*radius, center(2) + 1.5*radius];
    z_rng = [center(3) - 1.5*radius, center(3) + 1.5*radius];

    pc_roi = fcn_LiDARPoseEstimation_filterPointInXYZ(ptCloud_transformed, x_rng, y_rng, z_rng);
    
    
    
    scatter3(pc_roi(:,1), pc_roi(:,2), pc_roi(:,3), 10, 'r', 'filled');
    
    xyz_limits(1,:) = [min(x_rng(1), xyz_limits(1,1)), max(x_rng(2), xyz_limits(1,2))];
    xyz_limits(2,:) = [min(y_rng(1), xyz_limits(2,1)), max(y_rng(2), xyz_limits(2,2))];
    xyz_limits(3,:) = [min(z_rng(1), xyz_limits(3,1)), max(z_rng(2), xyz_limits(3,2))];

end

axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
legend('Irrelevant data', 'Identified sphere targets','Location','best');
% title(sprintf('LiDAR Targets in %s Frame - Scan %d', upper(view_frame), idx_scan_to_visualize));
grid on;
xyz_limits(:,1) = xyz_limits(:,1) - 1;
xyz_limits(:,2) = xyz_limits(:,2) + 1;
axis([xyz_limits(1,:), xyz_limits(2,:), xyz_limits(3,:)]);
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

end
