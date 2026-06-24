# Config

This folder contains the runtime YAML profiles for the `neuro_final_teleop` node.

```text
neuro_final_default.yaml   Normal operation values.
neuro_final_safe.yaml      Slower safety-focused values.
neuro_final_drilling.yaml  Drilling-practice values.
```

Each file is keyed by the ROS node name:

```yaml
neuro_final_teleop:
  ros__parameters:
    ...
```

When running the node directly, remap the node name or the parameters will not load:

```bash
ros2 run neuro_final_teleop neuro_final_teleop --ros-args \
  -r __node:=neuro_final_teleop \
  --params-file src/neuro_final_teleop/config/neuro_final_default.yaml
```

Use launch arguments, such as `robot_ip:=...`, for machine-specific values instead of editing the YAML for every computer.
