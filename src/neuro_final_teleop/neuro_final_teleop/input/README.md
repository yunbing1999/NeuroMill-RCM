# input

Input backends for NeuroFinal teleoperation.

`dualsense.py`

Primary input path. It reads a PS5 DualSense controller through `dualsense-controller`, maps PS5 buttons into the teleop input snapshot, and sends vibration/adaptive-trigger feedback when the `hidraw` device is writable.

Default PS5 mapping:

```text
Circle/R1    Deadman hold
Cross        Fixed-tip capture/release
Square       Force/torque tare
Triangle     Orthogonal alignment
L1           Initial pose action
Create       Fault recovery or quit
Left stick   Base-frame planar motion
R3 + right stick Fixed-tip entry/control
L2/R2        Depth out/in
D-pad up/down Speed scale
D-pad left/right J7 trim when sticks are idle
```

`gamepad.py`

Fallback pygame/SDL input layer for generic gamepads. The main NeuroFinal node uses the DualSense layer for normal operation.
