# External ROS Packages

This directory contains third-party ROS packages used by the full NeuroFinal workspace launch files.

- `xarm_description`: robot description/xacro files for xArm visualization.
- `zed-ros2-description`: ZED camera description and visualization assets.
- `zed-ros2-interfaces`: ZED ROS message and service interfaces.
- `zed-ros2-wrapper`: ZED camera wrapper, components, and launch files.

Application code belongs in `src/neuro_final_teleop`; external vendor packages should stay isolated here.

## Current workspace integration

xarm_description is required for the active RViz model. The robot-specific
calibration is config/kinematics/user/xarm7_kinematics_neuromill_check.yaml.

ZED packages are optional for the current RCM workflow. playback.launch.py still
references zed_description. system.launch.py has a commented ZED definition but
an active reference, so it is not a working combined startup.

Nested vendor READMEs are upstream documentation snapshots, not the current
NeuroMill startup guide. Several contain Humble-specific binary/Docker examples;
do not substitute those for this workspace's Jazzy instructions. They have not
been certified against current upstream releases or tested ZED hardware here.
OptiTrack integration is not provided by the ZED packages and is not yet
implemented in the main project.
