# Development Notes

These notes are for people changing NeuroFinal after the first setup works.

## Source Layout

The main package is `src/neuro_final_teleop`. Runtime Python code lives under `src/neuro_final_teleop/neuro_final_teleop`.

Use `src/external` only for third-party ROS packages required by launch files. Do not place project control logic there.

## Motion And Haptics

The current teleop path is PS5 DualSense input, xArm velocity control, fixed-tip mode, force/torque safety checks, and DualSense haptics.

Motion and haptic changes should be easy to review. Keep parameter names, topic names, safety states, and log messages clear enough that a new user can inspect a live run with ROS tools.

## Suggested Extension Pattern

- Add independent hardware bridges as separate ROS nodes.
- Add new tracking systems as separate packages under `src/` first.
- Publish TF frames, tracking quality, and diagnostics before coupling new sensors into teleop.
- Keep recorded-session support in mind: important live data should be publishable as ROS topics so the GUI recorder can capture it.

## Before A Live Test

1. Build from a clean terminal.
2. Source only `/opt/ros/humble/setup.bash` and `~/NeuroFinal/install/setup.bash`.
3. Confirm the robot IP, controller permissions, FT topic, and joint states.
4. Test haptic output with the tool away from contact.
5. Record a short bag and replay it before collecting important data.
