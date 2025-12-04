function datasets_cell = fcn_LiDARPoseEstimation_loadDatasets(folder_path)
% fcn_LiDARPoseEstimation_loadDatasets
% Loads all .mat files in a folder and constructs a cell array containing the specified variable
%
% INPUTS:
%   folder_path:   string, path to folder containing .mat files
%
% OUTPUTS:
%   datasets_cell: 1xN cell array, each cell contains the data loaded from one .mat file

% Get list of .mat files
mat_files = dir(fullfile(folder_path, '*.mat'));

% Preallocate cell array
datasets_cell = cell(numel(mat_files), 1);

% Load each .mat file
for idx = 1:numel(mat_files)
    fullpath = fullfile(folder_path, mat_files(idx).name);
    S = load(fullpath, 'dataStruct'); % Variable name saved in the file is 'dataStruct'
    
    
    datasets_cell{idx} = S.dataStruct;
    
       
end

fprintf('Loaded %d .mat files from folder: %s\n', numel(mat_files), folder_path);
end
