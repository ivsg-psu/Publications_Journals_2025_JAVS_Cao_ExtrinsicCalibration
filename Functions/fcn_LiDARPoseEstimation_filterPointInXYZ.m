function LiDAR_ptCloud_array_filtered = fcn_LiDARPoseEstimation_filterPointInXYZ(...
    LiDAR_pointcloud_array, x_range, y_range, z_range)
% fcn_LiDARPoseEstimation_filterPointInXYZ
% Filters 3D LiDAR point cloud based on specified X, Y, and Z value ranges.
%
% FORMAT:
%   LiDAR_ptCloud_array_filtered = fcn_LiDARPoseEstimation_filterPointInXYZ(...
%       LiDAR_ptCloud_array, x_range, y_range, z_range)
%
% INPUTS:
%   LiDAR_ptCloud_array: NxK array
%       Original point cloud, where first 3 columns are XYZ
%
%   x_range: 1x2 vector
%       [xmin, xmax] range to keep in X-axis
%
%   y_range: 1x2 vector
%       [ymin, ymax] range to keep in Y-axis
%
%   z_range: 1x2 vector
%       [zmin, zmax] range to keep in Z-axis
%
% OUTPUTS:
%   LiDAR_ptCloud_array_filtered: MxK array
%       Filtered point cloud within bounding box (M <= N)
%
% DEPENDENCIES:
%   None
%
% EXAMPLE:
%   pc_filtered = fcn_LiDARPoseEstimation_filterPointInXYZ(pc, [-1 1], [-2 2], [0 2]);

% Author: Xinyu Cao
% Date: 2025-07-16
% Contact: xfc5113@psu.edu

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
narginchk(4,4)
assert(size(LiDAR_pointcloud_array,1)>=3,'Point cloud array must have at least three columns ([X Y Z])')

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
xyz_array = LiDAR_pointcloud_array(:,1:3);
x_valid = (xyz_array(:,1) >= min(x_range)) & (xyz_array(:,1) <= max(x_range));
y_valid = (xyz_array(:,2) >= min(y_range)) & (xyz_array(:,2) <= max(y_range));
z_valid = (xyz_array(:,3) >= min(z_range)) & (xyz_array(:,3) <= max(z_range));

valid_mask = x_valid & y_valid & z_valid;
LiDAR_ptCloud_array_filtered = LiDAR_pointcloud_array(valid_mask, :);
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
