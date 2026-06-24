# control

Reusable control and safety modules for NeuroFinal teleoperation.

`base_node.py`

Common ROS node setup: parameters, xArm SDK connection, health checks, state transitions, velocity sending, tare action, alignment action, and shutdown.

`fixed_point_ik.py`

Joint-space fixed-point solver used by fixed-tip mode.

`ft_guard.py`

Force/torque guard that evaluates fresh/stale data, hard limits, warning slowdown, and inward-motion blocking.

`kinematics.py`

PyKDL xArm7 model used for forward kinematics, Jacobian calculations, and SDK/model validation.

`math_utils.py`

Small math helpers used by motion and safety code.

`tip_lock_controller.py`

Fixed-tip controller helper used by the active X/R3 constrained mode.

`safety.py`

Rate limiting, joint-risk checks, and safety helper classes.

`state_machine.py`

Explicit teleop state machine with allowed transitions for idle, free teleop, fixed-tip capture/active states, alignment, and fault recovery.
