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




