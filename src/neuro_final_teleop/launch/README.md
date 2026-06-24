# Launch Files

`teleop.launch.py`

Starts the normal live teleoperation stack: force/torque bridge, joint-state bridge, main teleop node, and session GUI.

`bridge_only.launch.py`

Starts only the xArm force/torque bridge and joint-state bridge. Use this when checking hardware topics before teleoperation.

`system.launch.py`

Starts the full visualization workflow: robot model, optional ZED launch, optional RViz, bridges, teleop, and GUI. RViz visualizes live or replayed ROS data; it is not a robot simulator.

`playback.launch.py`

Starts RViz, robot model publishing, ZED visualization support, and the session GUI using simulated ROS time. Use it together with `ros2 bag play --clock`.

## Common Commands

```bash
ros2 launch neuro_final_teleop teleop.launch.py robot_ip:=192.168.1.243
ros2 launch neuro_final_teleop system.launch.py robot_ip:=192.168.1.243
ros2 launch neuro_final_teleop bridge_only.launch.py robot_ip:=192.168.1.243
ros2 launch neuro_final_teleop playback.launch.py
```

The `teleop.launch.py` and `system.launch.py` files also accept `debug_topic_enable`, `trigger_enable`, and `rumble_enable` arguments for haptic/debug channel selection.
