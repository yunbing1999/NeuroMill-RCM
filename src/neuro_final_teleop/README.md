# neuro_final_teleop

Main ROS 2 Jazzy package for xArm7 DualSense teleoperation, fixed-tip and RCM
control, force feedback, basic CSV recording, and drill visualization.
See the [workspace README](../../README.md) for the current separate-terminal startup.

## Package map

| Path | Responsibility |
|---|---|
| neuro_final_teleop/neuro_final_teleop.py | Main node, parameters, controllers, diagnostics and capture events |
| neuro_final_teleop/motion_modes.py | Operator input, free/fixed-tip/RCM transitions and motion requests |
| neuro_final_teleop/force_haptics.py | Force processing, contact feedback and haptic output |
| neuro_final_teleop/control/ | KDL, solvers, state machine and protection helpers |
| neuro_final_teleop/input/ | DualSense backend and pygame fallback implementation |
| neuro_final_teleop/nodes/ | Independent bridge, GUI, logger and visualization nodes |
| neuro_final_teleop/drill_mesh.py | Continuous display-only drill mesh generation |
| neuro_final_teleop/assets/ | Controller-guide image |
| config/ | Default and alternative runtime parameter profiles |
| launch/ | Generic, visualization and playback launches |
| urdf/ | Calibrated robot/sensor/holder/drill display assembly |
| meshes/milling/ | Active static meshes |
| meshes/archive/ | Original reference meshes, not installed |
| rviz/ | RViz display configuration |
| experiments/ | SDK/KDL comparison tools; results go to rcm_logs/kinematics_validation |
| test/ | Unit and dry-run regression checks |

## Installed executable entry points

- neuro_final_teleop: main control node.
- ft_bridge: force/torque topic and tare service.
- joint_state_bridge: encoder feedback as joint states.
- session_gui: force monitoring, haptic controls and rosbag recording.
- rcm_diagnostics_logger: basic CSV and metadata per capture.
- rcm_visualizer: RCM entry coordinates, trajectory and summary markers.

## Current behavior

Controller TCP translation and rotation are used with the selected KDL model.
Software virtual-tip offsets and compact CSV output have been removed.
Insertion depth limits are disabled; insertion speed, joint velocity and
acceleration limits remain. Fixed-tip mode remains available.

Use rcm_sensor_view.launch.py for the current model. system.launch.py has an
unresolved ZED include reference and is not the recommended startup path.
Generic teleop.launch.py does not apply the full experiment overrides.
Playback uses ROS bags, not the basic CSV files.

Some historical depth-limit tests still require updates. Display geometry and
model-derived errors are not independent physical accuracy measurements.
