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

## Reproducing the analysis

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/neuro_final_teleop/experiments/analyze_studio_rcm_repeats.py
```

The analysis script regenerates `run_summary.csv` and also creates derived
steady-sample and plot files. Those generated files are not required to retain
the five original runs and may be removed after inspection.
