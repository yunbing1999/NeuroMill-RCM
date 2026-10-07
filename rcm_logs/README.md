# RCM Experiment Recordings

## Current directories

| Path | Contents |
|---|---|
| `new_tcp/basic/` | Basic recordings with the new TCP: 20 CSV files and 20 matching metadata JSON files, recorded on 2026-09-29, 2026-10-05, 2026-10-07 |
| `kinematics_validation/` | Kinematic validation: `sdk_kdl_fk_scan.csv` contains the earlier SDK/KDL comparison; `sdk_kdl_fk_scan_calibrated.csv` contains the calibrated KDL comparison |
| `archive/` | Earlier standard experiments, insertion demonstrations, and insertion pilot experiments; see the local `archive/README.md` for individual runs |

This inventory reflects local files as of 2026-10-07. Experiment data are not distributed with the repository; a fresh clone may not contain these directories or CSV files.

## Starting basic recording

Source the ROS 2 and workspace environments, then start the logger before entering RCM mode:

```bash
ros2 run neuro_final_teleop rcm_diagnostics_logger --ros-args \
  -p log_mode:=basic \
  -p output_dir:=/home/yunbing/NeuroMill_Final/rcm_logs/new_tcp/basic
```

Each successful RCM capture creates a CSV and a matching metadata JSON file. Exiting RCM closes the session; another capture creates new files. Ctrl+C closes any active recording.

## CSV fields

- `elapsed_s`: elapsed time since the first recorded sample, in seconds.
- `lateral_error_mm`: model-derived distance from the fixed entry point to the tool shaft axis, in millimetres.
- `tcp_x_mm`, `tcp_y_mm`, `tcp_z_mm`: model TCP position in the robot base frame, in millimetres.
- `q1_deg` through `q7_deg`: encoder feedback joint angles, in degrees.

The matching `_metadata.json` records the captured entry point, controller TCP, model configuration, units, and data sources. Keep it together with its CSV. Historical metadata preserves the configuration used at recording time and should not be rewritten to match the current code.

The current controller uses the configured physical TCP without an additional software virtual-tip offset. CSV RCM errors describe model consistency; independent optical measurements are needed to validate physical accuracy.

## File naming

`rcm_basic_20261005_151806_139375.csv` indicates the basic format, the date 2026-10-05, the time 15:18:06, and a microsecond component used to distinguish files. Its matching JSON shares the same timestamp.

## Git rules

Everything under `rcm_logs/` except this README is ignored by Git, including CSV files, metadata, plots, and archive documentation. Ignore rules do not delete local data.
