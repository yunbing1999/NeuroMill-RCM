# nodes

Independent ROS nodes used by the live and recording workflows.

`ft_bridge.py`

Connects to the xArm SDK, initializes the force/torque sensor, publishes `/xarm/ft_data`, and provides `/xarm/tare_sensor`.

`joint_state_bridge.py`

Connects to the xArm SDK and publishes `/joint_states` for robot visualization and recording.

`session_gui.py`

PyQt GUI for live force/torque monitoring, haptic evaluation controls, tare command, and rosbag recording.

The GUI recorder writes sessions under `~/NeuroFinal/recordings` unless `NEURO_FINAL_RECORDING_ROOT` is set.
