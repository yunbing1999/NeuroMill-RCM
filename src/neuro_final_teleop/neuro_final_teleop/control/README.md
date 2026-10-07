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

## RCM controller and current limits

rcm_controller.py implements captured entry-point geometry and a two-level
joint-velocity solver: lateral RCM correction first, operator rotation/insertion
second. Cumulative depth limits and boundary slowdown are disabled; speed and
joint acceleration/velocity limits remain active.

kinematics.py applies controller TCP translation and RPY rotation with the
selected KDL model. No software virtual-tip offset is added. Position validation
gates constrained modes; orientation mismatch is reported diagnostically.

The state machine includes RCM capture/active states. Fixed-tip functionality is
separate from RCM and remains supported. Historical depth-limit assertions in
test_rcm_controller.py still need updating to match the current behavior.
