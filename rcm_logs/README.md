# RCM Validation Data

This directory contains the retained kinematic-consistency scans and the five
standard Studio Sim RCM runs. Pilot, superseded, offline-generated, and failed
experiments have been removed.

CSV files are ignored by the repository's root `.gitignore`; they remain local
unless explicitly force-added or published separately as a dataset.

## Kinematic consistency

- `sdk_kdl_fk_scan.csv`
  - Nominal xArm7 KDL model compared with xArm SDK FK.
  - 143 configurations.
  - Position RMS: 10.120 mm.
  - Rotation RMS: 2.730 deg.

- `sdk_kdl_fk_scan_calibrated.csv`
  - Robot-specific calibrated KDL model compared with xArm SDK FK.
  - 143 configurations.
  - Position RMS: 0.027 mm.
  - Rotation RMS: 0.012 deg.

These results measure numerical consistency with the SDK, not physical
robot accuracy.

## Repeated Studio Sim RCM experiment

Formal data:

`studio_calibrated_standard/full_stick_0p5_deg_s/`

Protocol:

- Studio Sim mode.
- Physical robot remained stationary.
- Initial joints: `[0, -40, 0, 75, 0, 115.1, 1.5]` deg.
- Robot-specific calibrated KDL model.
- RCM insertion disabled.
- Right stick fully deflected in one direction.
- Command duration: approximately 5 seconds.
- Maximum angular speed: 0.5 deg/s.
- Speed scale: 0.75.
- Expected stable command: 0.375 deg/s.
- Stable samples selected above 0.36 deg/s.
- Five repeated runs.

Combined stable-period results:

- Samples: 625.
- Mean RCM error: 0.000189 mm.
- Combined RMS RCM error: 0.000284 mm.
- Maximum RCM error: 0.003061 mm.
- Limited samples: 0.

These RCM errors are calculated from calibrated KDL and SDK joint
feedback. They do not represent externally measured physical RCM
accuracy.

## Retained directory structure

```text
rcm_logs/
├── README.md
├── sdk_kdl_fk_scan.csv
├── sdk_kdl_fk_scan_calibrated.csv
└── studio_calibrated_standard/
    └── full_stick_0p5_deg_s/
        ├── run_01/ ... run_05/   # one raw CSV per run
        └── analysis/
            └── run_summary.csv
```

`run_summary.csv` contains one summary row for each retained run.

## Compact RCM logging

New experiments use one compact logger. Start the teleoperation node first and
confirm that `/neuro_final/rcm_diagnostics` is being published. Then open a
second terminal and run:

```bash
cd /home/yunbing/NeuroMill_Final
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run neuro_final_teleop rcm_diagnostics_logger --ros-args \
  -p output_dir:=/home/yunbing/NeuroMill_Final/rcm_logs/compact_runs/run_01
```

Press `Ctrl+C` in the logger terminal after the experiment. The logger prints a
summary and saves one timestamped CSV file in the selected output directory.

Each new compact CSV contains only these 13 fields:

```text
time_s
lateral_error_mm
desired_angular_speed_rad_s
achieved_angular_speed_rad_s
angular_error_rad_s
requested_insertion_mm_s
target_insertion_mm_s
achieved_insertion_mm_s
insertion_depth_mm
max_joint_speed_rad_s
travel_limited
joint_velocity_limited
joint_acceleration_limited
```

The three insertion values have different meanings:

- `requested`: operator command before travel limiting.
- `target`: command after slowdown and travel limiting.
- `achieved`: motion produced after the hierarchical solver and joint limits.

## Analyzing one compact CSV

From the workspace root, pass one compact CSV path to the analyzer:

```bash
cd /home/yunbing/NeuroMill_Final

python3 src/neuro_final_teleop/experiments/analyze_compact_rcm_csv.py \
  rcm_logs/compact_runs/run_01/rcm_YYYYMMDD_HHMMSS.csv
```

The analyzer reports sample count and rate, RCM mean/RMS/95th-percentile/maximum
error, mean angular and insertion tracking errors, insertion depth range,
maximum joint speed, and the three limiter counts. It reads the CSV without
modifying it or creating derived files.

The retained `studio_calibrated_standard` files use the previous detailed CSV
schema. They document the completed rotation-only experiment but are not input
for `analyze_compact_rcm_csv.py`.
