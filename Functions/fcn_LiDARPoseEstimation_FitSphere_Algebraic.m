function [center, radius, residualErrors] = ...
         fcn_LiDARPoseEstimation_FitSphere_Algebraic(XYZ_array)
% fcn_LiDARPoseEstimation_FitSphere_Algebraic
% Fits a sphere to 3D point cloud data using the algebraic least squares 
% method (closed-form solution). Outputs the estimated sphere center, 
% radius, and per-point residual errors.
%
% FORMAT:
%   [center, radius, residualErrors] = ...
%       fcn_LiDARPoseEstimation_FitSphere_Algebraic(XYZ_array)
%
% INPUTS:
%   XYZ_array: Nx3 array
%       A set of 3D points [X, Y, Z] that lie approximately on a sphere.
%
% OUTPUTS:
%   center: 1x3 vector
%       Estimated [X, Y, Z] center of the sphere.
%
%   radius: scalar
%       Estimated radius of the fitted sphere.
%
%   residualErrors: Nx1 vector
%       Absolute difference between each point’s distance to the center
%       and the estimated radius, i.e., residuals in meters.
%
% NOTES:
%   - This algebraic method provides a fast, closed-form fit to a sphere,
%     though it is more sensitive to outliers than geometric (nonlinear)
%     least-squares methods.
%   - Residual errors are calculated geometrically for interpretability.
%
% Author: Xinyu Cao  
% Revised: 2025-07-15

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
assert(size(XYZ_array,2)==3,'Input must have at least three columns')

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
% Extract XYZ
x = XYZ_array(:,1);
y = XYZ_array(:,2);
z = XYZ_array(:,3);

% Form A and b
A = [2*x, 2*y, 2*z, -ones(size(x))];
b = x.^2 + y.^2 + z.^2;

% Solve A*theta = b
theta = A \ b;

% Extract center and radius
xc = theta(1);
yc = theta(2);
zc = theta(3);
D = theta(4);

center = [xc, yc, zc];
radius = sqrt(xc^2 + yc^2 + zc^2 - D);

% Calculate residuals
distances = vecnorm(XYZ_array - center, 2, 2);
residualErrors = abs(distances - radius);

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
