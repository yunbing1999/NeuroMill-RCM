# NeuroMill Final

NeuroMill Final is a ROS 2 Jazzy workspace for teleoperating a UFACTORY xArm7
with a PS5 DualSense controller. It includes free Cartesian teleoperation,
fixed-tip control, two-level remote-center-of-motion (RCM) control, bounded
insertion and withdrawal, force/torque safety, haptic feedback, diagnostics,
and experiment logging.

The active ROS package is `neuro_final_teleop`.

> **Safety:** Develop and validate new motion features in UFACTORY Studio Sim
> first. Use low speed, keep the emergency stop accessible, and do not assume
> that a model-based RCM error is a physical measurement of the drill shaft.

## Current RCM Design

The RCM joint-velocity solver uses two priority levels:

```text
P1: maintain the captured RCM entry-point constraint
P2: operator rotation and insertion/withdrawal at equal priority
```

P2 is solved inside the null space of P1. Joint velocity and acceleration
limits are applied afterward. Insertion includes positive and negative travel
limits plus a soft slowdown zone near each boundary.

The current controller assumes that the drill shaft is parallel to the modeled
tool `+Z` axis. This assumption has not yet been fully validated against the
physical drill mounting and remains an important limitation.

## Repository Layout

```text
NeuroMill_Final/
├── README.md
├── requirements.txt
├── rcm_logs/                       RCM experiment documentation and local CSVs
└── src/
    ├── neuro_final_teleop/         Main ROS 2 package
    └── external/                   xArm and ZED ROS packages
```

Important package files:

```text
src/neuro_final_teleop/
├── config/                         Runtime parameter profiles
├── experiments/                    Offline analysis scripts
├── launch/                         ROS 2 launch files
├── neuro_final_teleop/
│   ├── neuro_final_teleop.py       Main node and parameters
│   ├── motion_modes.py             Operator commands and mode transitions
│   ├── force_haptics.py            FT safety and DualSense feedback
│   ├── control/
│   │   ├── kinematics.py           Calibrated PyKDL model
│   │   └── rcm_controller.py       Two-level RCM solver
│   └── nodes/
│       └── rcm_diagnostics_logger.py
└── test/                            Unit and dry-run tests
```

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3.12
- UFACTORY xArm Python SDK
- PyKDL
- PS5 DualSense controller
- Access to the xArm controller network

Optional components include the xArm force/torque sensor, RViz, and a ZED
camera with its SDK and ROS driver.

## First-Time Setup

```bash
cd ~
git clone git@github.com:yunbing1999/NeuroMill-RCM.git NeuroMill_Final
cd ~/NeuroMill_Final

source /opt/ros/jazzy/setup.bash

sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-pip \
  python3-pykdl \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rviz2 \
  ros-jazzy-xacro

python3 -m pip install -r requirements.txt
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

Verify the package:

```bash
ros2 pkg prefix neuro_final_teleop
```

The command should return a path under this workspace's `install/` directory.

## Build After Code Changes

```bash
cd /home/yunbing/NeuroMill_Final
source /opt/ros/jazzy/setup.bash

colcon build \
  --symlink-install \
  --packages-select neuro_final_teleop

source install/setup.bash
```

Every new terminal must source both ROS and the workspace:

```bash
source /opt/ros/jazzy/setup.bash
source /home/yunbing/NeuroMill_Final/install/setup.bash
```

## Hardware Checklist

Before connecting the teleoperation node:

- Confirm whether Studio is in **Sim** or **Real** mode.
- Confirm that the intended robot is reachable at `192.168.1.243`.
- Confirm that no other program is commanding the robot.
- Connect the DualSense and check that pygame can see it.
- Verify the active TCP offset and payload in UFACTORY Studio.
- Keep motion speeds low for the first run.
- Keep the emergency stop accessible.

Check the controller:

```bash
ls -l /dev/input/js*

python3 -c 'import pygame; pygame.init(); pygame.joystick.init(); print([(i, pygame.joystick.Joystick(i).get_name()) for i in range(pygame.joystick.get_count())])'
```

If DualSense haptic output lacks permission, install an appropriate `udev`
rule for Sony VID `054c`, PID `0ce6`, then reconnect the controller.

## Calibrated Kinematics and TCP

The robot-specific joint-origin parameters are stored in:

```text
src/external/xarm_description/config/kinematics/user/
xarm7_kinematics_neuromill_check.yaml
```

They were exported from the xArm controller through
`gen_kinematics_params.py`, which reads 42 controller-resident values
(`7 joints × [x, y, z, roll, pitch, yaw]`) over TCP port 502.

At runtime the YAML is loaded only into the local `KDLKinModel`; it is not
written back to the controller:

```text
xArm controller calibration
        ↓ export
robot-specific kinematics YAML
        ↓ load at node startup
local KDL FK and Jacobian
        ↓
Tip-Lock and RCM solvers
```

The active UFACTORY controller TCP offset is read separately from
`arm.tcp_offset`. Its XYZ translation is applied to local KDL FK and the tool
Jacobian.

Startup validation currently compares **TCP position only** between SDK and
KDL. A small validation error proves numerical consistency between the two
models at that pose; it does not prove physical drill-tip or drill-axis
accuracy. TCP orientation and the physical shaft direction require separate
multi-pose and physical validation.

## Recommended Studio Sim RCM Run

First put UFACTORY Studio in **Sim** mode. Then start the node with conservative
values. The calibrated kinematics path and controller TCP must match the setup
being tested.

```bash
cd /home/yunbing/NeuroMill_Final
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run neuro_final_teleop neuro_final_teleop --ros-args \
  --params-file /home/yunbing/NeuroMill_Final/src/neuro_final_teleop/config/neuro_final_default.yaml \
  -p robot_ip:=192.168.1.243 \
  -p kinematics_yaml:=/home/yunbing/NeuroMill_Final/src/external/xarm_description/config/kinematics/user/xarm7_kinematics_neuromill_check.yaml \
  -p require_simulation_robot:=false \
  -p v7_rcm_enable:=true \
  -p v7_rcm_insertion_enable:=true \
  -p v7_rcm_max_insertion_mm_s:=1.0 \
  -p v7_rcm_max_insertion_depth_mm:=5.0 \
  -p v7_rcm_max_withdrawal_depth_mm:=3.0 \
  -p v7_rcm_travel_slowdown_mm:=2.0 \
  -p v7_rcm_max_angular_deg_s:=0.5
```

`require_simulation_robot:=false` is used because the SDK has reported
`Controller simulation mode: False` even while Studio Sim was selected. Always
confirm the Studio **Sim/Real** selector visually before enabling motion.

## DualSense Controls

The current default deadman setting is `circle`; the input layer also accepts
R1 while this setting is active.

| Control | Free teleoperation | RCM mode |
|---|---|---|
| Hold Circle or R1 | Enable motion | Enable motion |
| Options | Capture and enter RCM | Exit RCM |
| Right stick | Cartesian orientation command | Tool-frame X/Y rotation |
| R2 | Positive/inward depth command | Insertion |
| L2 | Negative/outward depth command | Withdrawal |
| Cross | Enter Tip-Lock | No RCM action |
| R3 click | Toggle Tip-Lock when enabled | No RCM action |
| Square | FT tare | No mode change |
| Triangle | Orthogonal alignment | No mode change |
| L1 | Move to configured initial joint pose | Not handled until RCM is exited |
| D-pad up/down | Change speed scale | Change speed scale |
| D-pad left/right | J7 trim when allowed | — |
| Create | Recovery or quit, depending on state | Recovery or quit |

Important RCM behavior:

1. Move the tool tip to the intended entry point.
2. Press **Options** to capture that point and enter RCM.
3. Hold the deadman.
4. Use the right stick for constrained rotation.
5. Use R2/L2 for insertion/withdrawal when insertion is enabled.
6. Press **Options** again to exit RCM.

L1 uses `initial_pose_joint_deg_csv` unless
`initial_pose_use_startup:=true`. It is a joint-space target, not the RCM
capture pose.

## RCM Parameters

The RCM parameters are declared in `neuro_final_teleop.py`. Important defaults
are:

| Parameter | Default | Meaning |
|---|---:|---|
| `v7_rcm_enable` | `true` | Enable RCM mode |
| `v7_rcm_insertion_enable` | `false` | Allow L2/R2 motion in RCM |
| `v7_rcm_max_angular_deg_s` | `5.0` | Maximum operator rotation speed |
| `v7_rcm_max_insertion_mm_s` | `5.0` | Maximum insertion/withdrawal speed |
| `v7_rcm_max_insertion_depth_mm` | `20.0` | Positive travel limit from capture |
| `v7_rcm_max_withdrawal_depth_mm` | `10.0` | Negative travel limit from capture |
| `v7_rcm_travel_slowdown_mm` | `5.0` | Soft slowdown width near a limit |
| `v7_rcm_correction_gain_s` | `12.0` | P1 error correction gain |
| `v7_rcm_max_correction_mm_s` | `10.0` | Maximum P1 correction speed |
| `v7_rcm_damping` | `0.025` | Damped pseudoinverse parameter |
| `v7_rcm_qdot_limit_rad_s` | `0.40` | Joint velocity limit |
| `v7_rcm_qddot_limit_rad_s2` | `1.50` | Joint acceleration limit |

The characteristic length is currently calculated as:

```text
characteristic_length_m =
    (maximum insertion speed in m/s) / (maximum angular speed in rad/s)
```

Consequently, changing either maximum speed also changes the relative scaling
inside P2. This coupling is under evaluation and should be considered when
interpreting rotation-tracking results.

## Compact RCM Diagnostics

The main node publishes JSON diagnostics on:

```text
/neuro_final/rcm_diagnostics
```

Check one complete message:

```bash
ros2 topic echo \
  /neuro_final/rcm_diagnostics \
  std_msgs/msg/String \
  --once \
  --full-length
```

Start the compact logger in a second sourced terminal:

```bash
cd /home/yunbing/NeuroMill_Final
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run neuro_final_teleop rcm_diagnostics_logger --ros-args \
  -p output_dir:=/home/yunbing/NeuroMill_Final/rcm_logs/compact_runs/run_01
```

Stop it with `Ctrl+C`. It writes a timestamped CSV and prints a summary.

Analyze one compact CSV:

```bash
python3 src/neuro_final_teleop/experiments/analyze_compact_rcm_csv.py \
  rcm_logs/compact_runs/run_01/rcm_YYYYMMDD_HHMMSS.csv
```

See [rcm_logs/README.md](rcm_logs/README.md) for the 13 CSV fields, retained
experiments, metric definitions, and interpretation limits.

The reported lateral RCM error is computed from encoder joint feedback and the
same calibrated KDL model used by the controller. It is useful for controller
and model consistency, but it is not an independent physical measurement. A
physical RCM-accuracy claim requires an external reference such as optical
tracking or a measured entry-point fixture.

## Other Runtime Components

| Executable | Purpose |
|---|---|
| `neuro_final_teleop` | Main teleoperation and RCM node |
| `ft_bridge` | Publish xArm force/torque data |
| `joint_state_bridge` | Publish `/joint_states` |
| `session_gui` | FT/haptic monitoring and rosbag recording |
| `rcm_diagnostics_logger` | Record compact RCM CSV data |

The normal non-RCM launch is:

```bash
ros2 launch neuro_final_teleop teleop.launch.py \
  robot_ip:=192.168.1.243
```

### Full RViz/ZED launch status

`system.launch.py` is not currently the recommended startup path. Its ZED
include block is commented out while the launch description still references
`zed_camera_launch`. Fix and test that launch file before using it for a formal
experiment.

## Dry Run

Dry run checks imports, parameters, and node construction without connecting
to the robot:

```bash
cd /home/yunbing/NeuroMill_Final
source /opt/ros/jazzy/setup.bash
source install/setup.bash

NEURO_FINAL_DRY_RUN=1 \
ros2 run neuro_final_teleop neuro_final_teleop --ros-args \
  -p enable_control_timer:=false
```

Dry run does not validate robot networking, physical motion, TCP calibration,
or DualSense hardware behavior.

## Development Checks

Run before committing controller changes:

```bash
cd /home/yunbing/NeuroMill_Final
source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 -m compileall -q \
  src/neuro_final_teleop/neuro_final_teleop

ROS_LOG_DIR=/tmp/neuro_rcm_test_logs \
python3 -m pytest -q \
  src/neuro_final_teleop/test/test_rcm_controller.py \
  src/neuro_final_teleop/test/test_rcm_kdl.py \
  src/neuro_final_teleop/test/test_rcm_node_dry_run.py \
  src/neuro_final_teleop/test/test_rcm_state_flow.py

colcon build \
  --symlink-install \
  --packages-select neuro_final_teleop
```

## Known Limitations and Open Validation Work

- SDK-versus-KDL agreement proves model consistency, not absolute physical
  accuracy.
- Startup kinematic validation currently checks TCP XYZ only, not orientation.
- The solver currently assumes the physical shaft is aligned with tool `+Z`.
- The physical drill-axis offset must be measured and incorporated before
  claiming physical RCM accuracy.
- The characteristic length is coupled to the configured maximum insertion and
  rotation speeds.
- Combined rotation-and-insertion tests across a broad range of poses are still
  required.
- Real-robot tests must follow successful software and Studio Sim validation.
- `system.launch.py` requires repair before it can reliably launch the complete
  RViz/ZED stack.

## Git Workflow

Develop features on a branch rather than directly on `main`:

```bash
git switch main
git pull personal main
git switch -c <feature-branch>
```

Before pushing:

```bash
git diff --check
git status --short
```
