# README_DATA.md  
Last update: 2025-08-04 by Xinyu Cao  

This readme is based on the IVSG README template in the repository: Errata_Tutorials_ReadmeTemplate. The goal of a readme is to guide users by providing information on how to get started, installations, help, and other information about your data.

---

## 📂 Description

This folder contains LiDAR calibration and test datasets used for evaluating extrinsic calibration between a Velodyne LiDAR and GPS reference frame using spherical targets.  

Each dataset is stored as an individual `.mat` file containing a **single struct** variable named `data`. These files were generated from raw static LiDAR scans, processed and split using MATLAB scripts (e.g., `fcn_save_example_calibration_datasets.m`), and are loaded using `fcn_load_example_calibration_datasets.m`.  

The data is typically used as input to the full calibration pipeline (e.g., `Sphere Target Fitting and LiDAR Calibration`) to perform target fitting, transformation estimation, and validation.

---

## 🚀 Getting Started

To load all datasets in this folder as a cell array of structs, run:

```matlab
datasets_cell = fcn_load_example_calibration_datasets('Data/Calibration');
```

This will return a 1xN cell array, where each element corresponds to one .mat file in the folder.

### Dependencies

* Function required:
`fcn_load_example_calibration_datasets.m`
* Used by:
  `script_demo_LiDARCalibration`  
* Other requirements:
MATLAB R2021a or newer recommended
No additional toolboxes required for loading

### Installing

* The install instructios area should note:

1. Software: MATLAB R2021a or newer
2. Data folder: Data/Calibration/ or Data/Test/
3. First script to run: `script_demo_LiDARCalibration`

### Executing program
* To verify the integrity of the dataset:

```cpp
datasets_cell = fcn_load_example_calibration_datasets('Data/Calibration');

% View a sample
disp(datasets_cell{1});
```

If the data loads successfully and contains expected fields (e.g., GPS, LiDAR, etc.), the dataset is valid.

---
## Help

### Documentation

- For more information please go through the main README of the repo.
