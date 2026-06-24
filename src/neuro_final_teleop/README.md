# neuro_final_teleop

This is the main NeuroFinal ROS 2 package. It provides PS5 DualSense teleoperation, xArm connection, fixed-tip behavior, force/torque haptics, bridge nodes, GUI tools, configs, and launch files.

For first-time setup and normal run commands, start with the workspace-level `README.md`.

## Package Map

```text
neuro_final_teleop/
  package.xml
  setup.py
  config/
  launch/
  neuro_final_teleop/
    neuro_final_teleop.py
    motion_modes.py
    force_haptics.py
    control/
    input/
    nodes/
    assets/
  test/
```

## Entry Points

These are installed by `setup.py` and can be run with `ros2 run`.

```text
neuro_final_teleop    Main robot teleop node.
ft_bridge             Publishes xArm force/torque data and provides tare service.
joint_state_bridge    Publishes xArm joint states.
session_gui           Live monitor, haptic controls, and rosbag recorder.
```

## Launch Files

```text
launch/teleop.launch.py       FT bridge, joint bridge, teleop, and GUI.
launch/bridge_only.launch.py  FT bridge and joint bridge only.
launch/system.launch.py       Robot model, ZED, RViz, bridges, teleop, and GUI.
launch/playback.launch.py     RViz and GUI support for recorded bag playback.
```

## Config Files

```text
config/neuro_final_default.yaml   Normal operation values.
config/neuro_final_safe.yaml      Slower safety-focused values.
config/neuro_final_drilling.yaml  Drilling-practice values.
```

The YAML files are keyed by the ROS node name `neuro_final_teleop`. Direct `ros2 run` commands should either use that node name or remap `__node` to it.

## Runtime Responsibilities

`neuro_final_teleop.py` creates the ROS node, loads parameters, connects to xArm, starts the DualSense input layer, and wires the main timer loop.

`motion_modes.py` handles operator intent: base-frame free motion, R3/Cross fixed-tip entry, right-stick fixed-tip movement, speed scale changes, depth commands, and button actions.

`force_haptics.py` handles FT interpretation: filtering, tare logic, contact estimation, haptic output, debug publishing, and force guard behavior.

`control/` contains reusable math, kinematics, safety, state-machine, and fixed-point control pieces.

`input/` contains the PS5 DualSense layer and a pygame fallback input layer.

`nodes/` contains independent ROS nodes used during live runs and recorded-session workflows.
