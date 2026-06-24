# External ROS Packages

This directory contains third-party ROS packages used by the full NeuroFinal workspace launch files.

- `xarm_description`: robot description/xacro files for xArm visualization.
- `zed-ros2-description`: ZED camera description and visualization assets.
- `zed-ros2-interfaces`: ZED ROS message and service interfaces.
- `zed-ros2-wrapper`: ZED camera wrapper, components, and launch files.

Application code belongs in `src/neuro_final_teleop`; external vendor packages should stay isolated here.
