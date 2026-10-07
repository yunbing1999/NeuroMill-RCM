# Launch files

| File | Current purpose/status |
|---|---|
| teleop.launch.py | Generic FT bridge, joint bridge, teleop and GUI; not the full calibrated RCM experiment configuration |
| bridge_only.launch.py | FT and joint-state bridges only |
| rcm_sensor_view.launch.py | Current calibrated robot/sensor/holder/drill display via robot_state_publisher; no robot motion commands |
| playback.launch.py | Robot/ZED visualization, RViz and GUI using simulated ROS time; use with ros2 bag play --clock |
| system.launch.py | Legacy combined workflow; currently broken because zed_camera_launch is referenced while its definition is commented out |

Use the [workspace README](../../../README.md) for the complete current startup.
Do not run overlapping generic launches alongside the separate-terminal nodes.

## Model parameters

rcm_sensor_view.launch.py defaults: sensor_height_m=0.056,
sensor_diameter_m=0.072, mount_yaw_deg=-78.382, shaft_length_m=0.10,
shaft_diameter_m=0.005. It selects neuromill_check kinematics, generates a temporary
continuous drill mesh and loads the display xacro.

Mount yaw and drill shape are approximate display geometry. TCP is a fixed
snapshot in the xacro and is not automatically read from the controller.

Playback still requires ZED description assets and does not replay basic CSV.
The generic teleop/system definitions expose debug_topic_enable, trigger_enable
and rumble_enable arguments; this does not resolve the system launch error.
