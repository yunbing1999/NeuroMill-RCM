# NeuroMill Final

NeuroMill Final is a standalone ROS 2 Humble workspace for xArm7 teleoperation
with a PS5 DualSense controller, force/torque haptic feedback, live monitoring,
RViz robot visualization, ZED camera recording, and rosbag playback.

The active ROS package is `neuro_final_teleop`.

After building and sourcing this workspace, this command should return a path:

```bash
ros2 pkg prefix neuro_final_teleop
```

If it says `Package not found`, the workspace has not been built and sourced in
the current terminal.

## Repository Layout

```text
NeuroMill_Final/
  README.md
  requirements.txt
  docs/
  recordings/                 Local bag output folder, ignored by git.
  src/
    neuro_final_teleop/       Main control package, GUI, launch files, configs.
    external/                 xArm and ZED ROS packages used by full launch.
```

The control code is in `src/neuro_final_teleop`. The `src/external` folder
contains ROS packages needed for robot description, ZED messages, ZED launch
support, RViz, and playback visualization.

## Runtime Pieces

| Component | Executable | Purpose |
|---|---|---|
| Teleop node | `neuro_final_teleop` | DualSense input, xArm velocity commands, fixed-tip mode, FT safety, haptics |
| FT bridge | `ft_bridge` | Reads xArm force/torque data and publishes `/xarm/ft_data` |
| Joint bridge | `joint_state_bridge` | Reads xArm joints and publishes `/joint_states` |
| Session GUI | `session_gui` | Live FT monitor, haptic mode control, recording button |
| Full launch | `system.launch.py` | Robot model, ZED, RViz, bridges, teleop, GUI |

Important topics:

| Topic | Purpose |
|---|---|
| `/xarm/ft_data` | Force/torque stream used by GUI, haptics, and bags |
| `/neuro_final/ft_haptic_debug` | JSON debug state for haptic/contact decisions |
| `/joint_states` | Robot joint states for RViz and playback |
| `/tf`, `/tf_static` | Robot/camera transform tree |
| `/zed/zed_node/rgb/color/rect/image` | ZED RGB image recorded with force data |
| `/zed/zed_node/rgb/color/rect/camera_info` | ZED camera calibration metadata |

## First Time Setup

Use Ubuntu 22.04 with ROS 2 Humble.

```bash
cd ~
git clone https://github.com/Isaac-Can-Do/NeuroMill_Final.git
cd ~/NeuroMill_Final

sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-pip \
  python3-pykdl \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2 \
  ros-humble-xacro

python3 -m pip install -r requirements.txt

source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Live ZED camera use also requires the ZED SDK/driver to be installed on the
computer and the camera to be connected. The ROS wrapper sources are included,
but the hardware driver still has to work on the machine.

## Hardware Checklist

Before live teleoperation:

- xArm controller is reachable on the network. The normal robot IP is
  `192.168.1.243`.
- The PS5 DualSense is connected.
- The user has read/write permission for the DualSense `hidraw` device.
- The xArm force/torque sensor is enabled.
- For camera recording, the ZED camera is connected and visible to the ZED SDK.
- No other process is commanding the same robot.

DualSense udev rule if input or haptics are blocked:

```bash
sudo tee /etc/udev/rules.d/70-dualsense.rules >/dev/null <<'EOF'
KERNEL=="hidraw*", ATTRS{idVendor}=="054c", ATTRS{idProduct}=="0ce6", MODE="0666", TAG+="uaccess"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Unplug and reconnect the controller after applying the rule.

## Normal Drilling Run

This starts the FT bridge, joint bridge, teleop node, and session GUI with the
Drilling profile:

```bash
cd ~/NeuroMill_Final
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch neuro_final_teleop teleop.launch.py \
  robot_ip:=192.168.1.243 \
  config:=$(ros2 pkg prefix neuro_final_teleop)/share/neuro_final_teleop/config/neuro_final_drilling.yaml \
  debug_topic_enable:=true \
  trigger_enable:=true \
  rumble_enable:=true
```

Use this when you want robot control, force feedback, haptic debug data, and
recording from the GUI, but do not need RViz or live ZED launch.

## Full RViz + ZED Run

Use this when you want to see the robot in RViz, start the ZED camera, run the
GUI, and record force values together with camera images:

```bash
cd ~/NeuroMill_Final
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch neuro_final_teleop system.launch.py \
  robot_ip:=192.168.1.243 \
  config:=$(ros2 pkg prefix neuro_final_teleop)/share/neuro_final_teleop/config/neuro_final_drilling.yaml \
  debug_topic_enable:=true \
  trigger_enable:=true \
  rumble_enable:=true
```

`system.launch.py` starts:

- `robot_state_publisher` for the xArm7 model
- ZED camera launch with `camera_model:=zed2i`
- RViz
- `ft_bridge`
- `joint_state_bridge`
- `neuro_final_teleop`
- `session_gui`

Useful variants:

```bash
# Open RViz/ZED/GUI/bridges without commanding the robot.
ros2 launch neuro_final_teleop system.launch.py \
  robot_ip:=192.168.1.243 \
  start_teleop:=false

# Run without the ZED camera.
ros2 launch neuro_final_teleop system.launch.py \
  robot_ip:=192.168.1.243 \
  start_zed:=false

# Use another ZED model if needed.
ros2 launch neuro_final_teleop system.launch.py \
  robot_ip:=192.168.1.243 \
  camera_model:=zed2i
```

If RViz opens but the image panel is empty, check that the ZED topic is
publishing:

```bash
ros2 topic hz /zed/zed_node/rgb/color/rect/image
```

You can inspect the live camera separately:

```bash
rqt_image_view /zed/zed_node/rgb/color/rect/image
```

## Controller Behavior

The normal controller is a PS5 DualSense.

| Control | Action |
|---|---|
| Hold Circle or R1 | Deadman; robot motion is allowed only while held |
| R3 click | While holding deadman, click once to enter fixed-tip control |
| Right stick | Fixed-tip angular control after R3/fixed-tip entry |
| Cross | Capture/release fixed-tip mode as a shortcut |
| Square | Force/torque tare |
| Triangle | Orthogonal alignment action |
| L1 | Move to the configured initial pose when enabled |
| L2 / R2 | R2 inward/insertion, L2 outward/extraction |
| Left stick | Base-frame X/Y planar motion in free mode |
| D-pad up/down | Increase/decrease speed scale |
| D-pad left/right | J7 trim when sticks are idle |
| Create | Fault recovery, or quit when enabled |

Frame convention:

- Outside fixed-tip mode, X/Y/Z motion is base-frame motion.
- Inside fixed-tip mode, right-stick angular control is tool-frame motion.
- With the current insertion setting, R2 moves base `-Z` in free mode and
  end-effector `+Z` in fixed-tip mode.

## Config Profiles

Config files live in `src/neuro_final_teleop/config/`.

| Config | Use case |
|---|---|
| `neuro_final_default.yaml` | Normal teleop and balanced haptic settings |
| `neuro_final_safe.yaml` | Slower first tests and close setup work |
| `neuro_final_drilling.yaml` | Drilling-style insertion tests with higher FT limits and drilling haptic tuning |

Run another profile:

```bash
ros2 launch neuro_final_teleop teleop.launch.py \
  robot_ip:=192.168.1.243 \
  config:=$(ros2 pkg prefix neuro_final_teleop)/share/neuro_final_teleop/config/neuro_final_safe.yaml
```

## Run Individual Nodes

Use these commands when debugging one part at a time.

```bash
# Teleop node only
ros2 run neuro_final_teleop neuro_final_teleop --ros-args \
  -r __node:=neuro_final_teleop \
  --params-file src/neuro_final_teleop/config/neuro_final_default.yaml \
  -p robot_ip:=192.168.1.243

# Force/torque bridge only
ros2 run neuro_final_teleop ft_bridge --ros-args \
  -p robot_ip:=192.168.1.243 \
  -p publish_hz:=100.0

# Joint-state bridge only
ros2 run neuro_final_teleop joint_state_bridge --ros-args \
  -p robot_ip:=192.168.1.243

# Session GUI only
ros2 run neuro_final_teleop session_gui
```

## Dry Run

Dry run starts the teleop node without connecting to the robot.

```bash
ros2 launch neuro_final_teleop teleop.launch.py dry_run:=true
```

For a headless smoke test:

```bash
ros2 run neuro_final_teleop neuro_final_teleop --ros-args \
  -p dry_run:=true \
  -p enable_control_timer:=false
```

Dry run checks Python imports, ROS parameters, node startup, config loading, and
topic/service setup. It does not test robot networking, ZED hardware, or
DualSense haptic write access.

## Session GUI And Recording

The session GUI is started by `teleop.launch.py`, `system.launch.py`, or:

```bash
ros2 run neuro_final_teleop session_gui
```

Important GUI features:

| GUI item | Purpose |
|---|---|
| Real-time FT values | Shows force/torque from `/xarm/ft_data` |
| Live plots | Tracks force/torque over time |
| Reset/Tare | Calls the FT tare/reset service |
| Haptic mode buttons | Switches between both, vibration only, trigger only, or off |
| Backend FT/Haptic Status | Shows debug state from `/neuro_final/ft_haptic_debug` |
| Contact Debug Timeline | Shows selected force, haptic ratio, and contact state |
| Free-space Noise Band | Runs a short calibration for contact tuning |
| Start Recording | Starts `ros2 bag record` with FT, haptic debug, joints, TF, and ZED image topics |

By default, recordings are saved under:

```text
~/NeuroFinal/recordings
```

Change the output folder before launching the GUI:

```bash
export NEURO_FINAL_RECORDING_ROOT=/path/to/recordings
```

The GUI records:

```text
/xarm/ft_data
/neuro_final/ft_haptic_debug
/joint_states
/tf
/tf_static
/zed/zed_node/rgb/color/rect/image
/zed/zed_node/rgb/color/rect/camera_info
```

A rosbag records only topics that are publishing at that moment. Before an
important test, check:

```bash
ros2 topic hz /xarm/ft_data
ros2 topic echo /neuro_final/ft_haptic_debug --once
ros2 topic hz /joint_states
ros2 topic hz /zed/zed_node/rgb/color/rect/image
```

Manual recording command:

```bash
mkdir -p recordings
ros2 bag record \
  -o recordings/manual_$(date +%Y_%m_%d_%H_%M_%S) \
  /xarm/ft_data \
  /neuro_final/ft_haptic_debug \
  /joint_states \
  /tf \
  /tf_static \
  /zed/zed_node/rgb/color/rect/image \
  /zed/zed_node/rgb/color/rect/camera_info
```

Stop manual recording with `Ctrl+C`.

## Playback

Playback uses the `bag` folder inside a session folder. Do not play the session
folder itself.

Correct folder shape:

```text
recordings/
  surgery_YYYY_MM_DD_HH_MM_SS/
    bag/
      metadata.yaml
      bag_0.db3
```

Start playback visualization:

```bash
cd ~/NeuroMill_Final
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch neuro_final_teleop playback.launch.py
```

In another sourced terminal, play a bag:

```bash
cd ~/NeuroMill_Final
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 bag play recordings/<session_folder>/bag --loop --clock
```

Example with an absolute path:

```bash
ros2 bag play /home/islam/NeuroFinal/recordings/surgery_2026_06_24_18_49_14/bag --loop --clock
```

If your ROS 2 version treats `--clock` as an option that expects a number, use
an explicit clock frequency:

```bash
ros2 bag play --loop --clock 100 /home/islam/NeuroFinal/recordings/surgery_2026_06_24_18_49_14/bag
```

Common mistakes:

```bash
# Wrong: --loop became part of the path.
ros2 bag play /home/islam/NeuroFinal/recordings/--loop surgery_2026_06_24_18_49_14

# Wrong: this is the session folder, not the actual bag folder.
ros2 bag play --loop /home/islam/NeuroFinal/recordings/surgery_2026_06_24_18_49_14

# Correct:
ros2 bag play /home/islam/NeuroFinal/recordings/surgery_2026_06_24_18_49_14/bag --loop --clock
```

Inspect bag contents:

```bash
ros2 bag info recordings/<session_folder>/bag
```

View only the ZED image while the bag is playing:

```bash
rqt_image_view /zed/zed_node/rgb/color/rect/image
```

## Live Runtime Checks

Run these in another sourced terminal:

```bash
ros2 node list
ros2 topic list | sort
ros2 topic hz /xarm/ft_data
ros2 topic echo /neuro_final/ft_haptic_debug --once
ros2 topic hz /joint_states
ros2 topic hz /zed/zed_node/rgb/color/rect/image
```

Expected nodes during a full system run include:

```text
/ft_bridge
/joint_state_bridge
/neuro_final_teleop
/session_gui
/robot_state_publisher
/rviz2
```

## Troubleshooting

### Package Not Found

```bash
cd ~/NeuroMill_Final
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg prefix neuro_final_teleop
```

### Robot Does Not Connect

```bash
ping -c 3 192.168.1.243
```

Then relaunch with the same IP:

```bash
ros2 launch neuro_final_teleop teleop.launch.py robot_ip:=192.168.1.243
```

### GUI Has No Force Data

```bash
ros2 topic hz /xarm/ft_data
```

If it is missing, run `teleop.launch.py`, `system.launch.py`, or `ft_bridge`.

### Haptic Debug Is Missing

```bash
ros2 topic echo /neuro_final/ft_haptic_debug --once
```

If nothing arrives, launch with:

```bash
debug_topic_enable:=true
```

### ZED Image Is Missing

Check the camera and topic:

```bash
ros2 topic list | grep zed
ros2 topic hz /zed/zed_node/rgb/color/rect/image
```

If no image topic exists, verify the ZED SDK can see the camera and relaunch
`system.launch.py`.

### Bag Does Not Play

Check that you are pointing to the inner `bag` folder:

```bash
find ~/NeuroFinal/recordings -maxdepth 3 -name metadata.yaml -print
ros2 bag info /home/islam/NeuroFinal/recordings/<session_folder>/bag
```

Then play:

```bash
ros2 bag play /home/islam/NeuroFinal/recordings/<session_folder>/bag --loop --clock
```

## Development Checks

Before pushing code changes:

```bash
python3 -m compileall -q src/neuro_final_teleop/neuro_final_teleop
colcon list
```

For a fuller local check:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```
