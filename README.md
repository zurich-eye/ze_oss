# Trajectory Analysis

A series of scripts and tools to compare trajectories among each other.

## Inputs

### Trajectories

Provided as CSV-file with the structure:
```
# timestamp, x, y, z, qx, qy, qz, qw
```
Where the  `timestamp` is in nanoseconds; `x`, `y`, `z` are metric and `qx`, `qy`, `qz`, `qw` are the elements of a unit quaternion.

### Hand-Eye Calibration
Specified as yaml file with the keys:
```
T_sensor_trackable:
  qx: ...
  qy: ... 
  qz: ... 
  qw: ...
  tx: ...
  ty: ...
  tz: ...
```
Again, `tx`, `ty` and `tz` are the matric translational values and `qx`, `qy`, `qz`, `qw` represent a unit quaternion. The transformation matrix described by `t_` and `q_` is `T_B_H`, transforming point from the frame of the "Hand" to the body-frame.

### Naming conventions in data directory:
The scripts expect a rather strict naming convention in the data directory:
* `traj_gt.csv`: the groundtruth trajectory,
* `traj_es.csv`: the estimated trajectory to evaluate,
* `traj_es_gt_matches.csv`: [optional] if provided it gives matched timestamps between the groundtruth and the estimate, and stores them in the file.

## Common Tasks

### Trajectory Analysis (`analyse.py`)

Takes a results folder with groundtruth and estimated trajectories, aligns them and generates a series of statistics and plots.

Command line arguments:
```
--data_dir: Full path to the directory containing the results
--format_gt: Format of the provided ground-truth [swe, pose, euroc] (Defalt: pose)
--format_es: Format of the estimate [swe, pose, euroc] (Default: pose)
--alignment: Trajectory alignment to perform (Default: se3):
             - se3: Estimate a 3d transform that minimizes the squared error between the groundtruth and estimated trajectory.
             - sim3: Estimate a 3d transform and a scaling factor that minimizes the squared error between the groundtruth and estimated trajectory.
             - first_frame: only align the starting points of the two trajectories.
--alignment_first_frame: the first frame to consider for trajectory alignment (Default: 0)
--alignment_last_frame: the last frame to consider for trajectory alignment, -1 for disabled. (Default: -1)
--plot_size: Scaling factor for the plots (Default: 0.2)
--skip_frames: Number of frames to skip between segment evaluations (Default: 1)
```

### Compare two results folders (`compare.py`)

Takes two results folders performs a comparative analysis on the two.

Command line arguments:
```
--trace_dir: full path to the results to compare to the reference
--reference: full path to the results that serve as reference for trace
```

## Relative errors (`relative_errors.py`)

Compure relative errors of two trajectories with respect to different segment lengths along the trajectory.

Command line arguments:
```
--data_dir: Full path to the directory containing the results
--format_gt: Format of the provided ground-truth [swe, pose, euroc] (Defalt: pose)
--format_es: Format of the estimate [swe, pose, euroc] (Default: pose)
--skip_frames: Number of frames to skip between segment evaluations (Default: 1)
--segment_lengths: The segments lengths to split the trajectory into for statistical evaluations as comma separated string (Default: 10, 40, 90, 160)
--segment_align_lsq: Align every segment pair to minimize the squared errors (on SE3)
--segment_align_lsq_translation_only:  Align every segment pair considering only translations
--plot_size: Scaling factor for the plots (Default: 0.2)
--plot_errors_along_trajectory: Plot the relative errors along the trajectory (Default: False)
```


