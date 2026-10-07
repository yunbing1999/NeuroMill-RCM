# nodes

Independent ROS nodes used by the live and recording workflows.

`ft_bridge.py`

Connects to the xArm SDK, initializes the force/torque sensor, publishes `/xarm/ft_data`, and provides `/xarm/tare_sensor`.

`joint_state_bridge.py`

Connects to the xArm SDK and publishes `/joint_states` for robot visualization and recording.

`session_gui.py`

PyQt GUI for live force/torque monitoring, haptic evaluation controls, tare command, and rosbag recording.

The GUI recorder writes sessions under `~/NeuroFinal/recordings` unless `NEURO_FINAL_RECORDING_ROOT` is set.

## RCM recording and visualization

rcm_diagnostics_logger.py supports only basic output (log_mode:=basic remains
accepted). Start before capture: /neuro_final/rcm_log_event opens/closes sessions,
and /neuro_final/rcm_diagnostics supplies samples. Each session produces CSV and
metadata JSON. See the workspace rcm_logs/README.md for fields.

rcm_visualizer.py publishes /neuro_final/rcm_markers in link_base, converting
millimetres to metres. It shows the captured entry XYZ, TCP, trajectory and error
statistics. On pause/exit it retains the last snapshot while the node runs.
The displayed drill geometry is separately supplied by RobotModel.

ft_bridge auto-zero is enabled by default. The GUI displays bridge data; main
teleop implements controller haptics. GUI rosbag recording is independent of
the basic CSV logger.
