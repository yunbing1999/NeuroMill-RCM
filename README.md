# NeuroMill Final

NeuroMill Final is a ROS 2 Jazzy workspace for teleoperating a UFACTORY xArm7
with a PS5 DualSense controller. It includes free Cartesian teleoperation,
fixed-tip control, two-level remote-center-of-motion (RCM) control, speed-limited
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
limits are applied afterward. Insertion/withdrawal speed is limited, but cumulative
depth limits and depth-boundary slowdown are currently disabled.

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
├── experiments/                    SDK/KDL validation scripts
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
  python3-venv \
  python3-rosdep \
  python3-pykdl \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rviz2 \
  ros-jazzy-xacro

# Use a ROS-compatible Python environment. On Ubuntu 24.04, a venv avoids
# modifying the externally managed system Python installation.
python3 -m venv --system-site-packages ~/venvs/neuromill
source ~/venvs/neuromill/bin/activate
python3 -m pip install -r requirements.txt
rosdep update
rosdep install --from-paths src/neuro_final_teleop src/external/xarm_description --ignore-src -r -y

colcon build --packages-select xarm_description neuro_final_teleop
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

Every new terminal must source both ROS and the workspace. If installed into the
venv above, activate it in each terminal as well (before building/running):

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

At runtime teleop loads this YAML into the local `KDLKinModel`; the current
visualization launch also selects this calibration. Neither writes it back to
the controller:

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
Jacobian. TCP RPY is also applied to the modeled tool orientation and shaft axis.
No additional virtual-tip offset is supported.

Startup validation gates constrained modes using TCP position agreement between
SDK and KDL, and separately reports orientation error as a diagnostic. A small validation error proves numerical consistency between the two
models at that pose; it does not prove physical drill-tip or drill-axis
accuracy. TCP orientation and the physical shaft direction require separate
multi-pose and physical validation.

## Current RCM Experiment Startup

Stop duplicate nodes before starting. In every terminal:

```bash
cd /home/yunbing/NeuroMill_Final
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

If using the first-time setup venv, also activate ~/venvs/neuromill.
The following commands use the real robot configuration. They do not enable
simulation. Match the controller TCP and the display model:
XYZ = [-36.87, -7.58, 243.61] mm; RPY = [3.52, 2.74, 6.27] degrees.

Run each command in a separate sourced terminal, in this order:

1. Joint feedback:

   ```bash
   ros2 run neuro_final_teleop joint_state_bridge --ros-args -p robot_ip:=192.168.1.243
   ```

2. Calibrated robot, sensor, holder and approximate drill model:

   ```bash
   ros2 launch neuro_final_teleop rcm_sensor_view.launch.py mount_yaw_deg:=-78.382 shaft_length_m:=0.10 shaft_diameter_m:=0.005
   ```

3. RViz:

   ```bash
   rviz2 -d /home/yunbing/NeuroMill_Final/src/neuro_final_teleop/rviz/neuro_final.rviz
   ```

4. Force/torque bridge (auto-zero is enabled by default; start without contact):

   ```bash
   ros2 run neuro_final_teleop ft_bridge --ros-args -p robot_ip:=192.168.1.243 -p publish_hz:=100.0
   ```

5. RCM markers and summary:

   ```bash
   ros2 run neuro_final_teleop rcm_visualizer --ros-args -p max_trail_points:=15000
   ```

6. Basic logger, before capturing RCM:

   ```bash
   ros2 run neuro_final_teleop rcm_diagnostics_logger --ros-args -p log_mode:=basic -p output_dir:=/home/yunbing/NeuroMill_Final/rcm_logs/new_tcp/basic
   ```

7. Main teleop:

   ```bash
   ros2 run neuro_final_teleop neuro_final_teleop --ros-args \
     -r __node:=neuro_final_teleop \
     --params-file /home/yunbing/NeuroMill_Final/src/neuro_final_teleop/config/neuro_final_default.yaml \
     -p robot_ip:=192.168.1.243 \
     -p kinematics_yaml:=/home/yunbing/NeuroMill_Final/src/external/xarm_description/config/kinematics/user/xarm7_kinematics_neuromill_check.yaml \
     -p require_simulation_robot:=false \
     -p v7_speed_scale_default:=0.75 \
     -p v7_rcm_enable:=true \
     -p v7_rcm_insertion_enable:=true \
     -p v7_rcm_max_insertion_mm_s:=5.0 \
     -p v7_rcm_max_angular_deg_s:=2.0
   ```

8. Optional force-monitoring GUI:

   ```bash
   ros2 run neuro_final_teleop session_gui
   ```

Release the deadman and exit RCM before stopping teleop and the logger.
Each capture gets its own CSV/metadata pair. GUI rosbag recording is separate.

The drill mesh is approximate display geometry, not physical metrology.
The display TCP is a fixed calibration snapshot; changing the controller TCP
requires updating the visual model too. TF yellow arrows connect coordinate
origins, not physical parts; disable TF > Show Arrows to hide them.

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
| `v7_rcm_correction_gain_s` | `12.0` | P1 error correction gain |
| `v7_rcm_max_correction_mm_s` | `10.0` | Maximum P1 correction speed |
| `v7_rcm_damping` | `0.025` | Damped pseudoinverse parameter |
| `v7_rcm_qdot_limit_rad_s` | `0.40` | Joint velocity limit |
| `v7_rcm_qddot_limit_rad_s2` | `1.50` | Joint acceleration limit |

Legacy depth-limit and travel-slowdown parameters remain declared, but the
current depth limiter passes commands through; do not rely on those parameters
to stop travel. Speed and joint limits remain active.

The characteristic length is currently calculated as:

```text
characteristic_length_m =
    (maximum insertion speed in m/s) / (maximum angular speed in rad/s)
```

Consequently, changing either maximum speed also changes the relative scaling
inside P2. This coupling is under evaluation and should be considered when
interpreting rotation-tracking results.

## Basic RCM Diagnostics

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

Start the basic logger before capturing RCM in a sourced terminal:

    ros2 run neuro_final_teleop rcm_diagnostics_logger --ros-args -p log_mode:=basic -p output_dir:=/home/yunbing/NeuroMill_Final/rcm_logs/new_tcp/basic

Each successful RCM capture creates a basic CSV and metadata JSON. Exiting RCM
closes the session; Ctrl+C closes any active file. See
[rcm_logs/README.md](rcm_logs/README.md) for the schema and data-source conventions.

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
| `rcm_diagnostics_logger` | Record capture-scoped basic RCM CSV data |
| `rcm_visualizer` | Publish captured entry coordinates, trajectory and error summary |

The generic teleop launch is below. It does not reproduce the calibrated RCM
experiment overrides above and does not start the RCM logger or visualizer:

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

The full controller test file still contains historical depth-limit assertions
that conflict with the current unlimited-depth behavior. These tests need updating;
a full-suite pass is not currently claimed. The basic CSV and visualizer tests are
in test_basic_csv.py and test_rcm_visualizer.py.

## Known Limitations and Open Validation Work

- SDK-versus-KDL agreement proves model consistency, not absolute physical
  accuracy.
- Startup validation gates on TCP position; orientation error is diagnostic only.
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

Experiment outputs under rcm_logs/ are ignored by Git except rcm_logs/README.md.
Keep raw CSV and metadata pairs together locally.
