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

The main node already uses the matching name neuro_final_teleop. The explicit
remap below is optional; if you choose a different node name, update the YAML key:

```bash
ros2 run neuro_final_teleop neuro_final_teleop --ros-args \
  -r __node:=neuro_final_teleop \
  --params-file src/neuro_final_teleop/config/neuro_final_default.yaml
```

Use launch arguments, such as `robot_ip:=...`, for machine-specific values instead of editing the YAML for every computer.

The current workflow uses neuro_final_default.yaml with explicit calibrated-KDL
and RCM speed overrides from the workspace README. Default stick signs are
lx=-1, ly=+1, rx=-1, ry=-1. Alternative profiles are not automatically merged,
and the safe profile name does not guarantee lower values for every parameter.
Virtual-tip parameters are no longer supported.
