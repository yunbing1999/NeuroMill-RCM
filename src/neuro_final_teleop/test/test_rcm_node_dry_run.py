import rclpy
import math

from neuro_final_teleop.control.state_machine import TeleopState
from neuro_final_teleop.neuro_final_teleop import NeuroFinalTeleopNode


def test_rcm_full_node_dry_run(monkeypatch):
    """Test capture, mode 4, joint command, and safe exit."""

    monkeypatch.setenv("NEURO_FINAL_DRY_RUN", "1")

    rclpy.init(args=[
        "--ros-args",
        "-p", "enable_control_timer:=false",
        "-p", "v7_rcm_max_insertion_mm_s:=2.0",
        "-p", "v7_rcm_max_angular_deg_s:=4.0",
        "-p", "v7_rcm_max_insertion_depth_mm:=12.0",
        "-p", "v7_rcm_max_withdrawal_depth_mm:=7.0",
        "-p", "v7_rcm_travel_slowdown_mm:=3.0",
    ])
    node = None

    try:
        node = NeuroFinalTeleopNode()
        node._constrained_modes_enabled = True

        cfg = node._v7_rcm_controller.cfg
        assert cfg.max_insertion_depth_mm == 12.0
        assert cfg.max_withdrawal_depth_mm == 7.0
        assert cfg.travel_slowdown_mm == 3.0
        expected_characteristic_length_m = (
            2.0 * 0.001 / math.radians(4.0)
        )

        assert math.isclose(
            node._v7_rcm_controller.cfg.characteristic_length_m,
            expected_characteristic_length_m,
            rel_tol=1e-12,
            abs_tol=1e-12,
        )

        joint_commands = []

        # Record fake mode-4 commands without touching hardware.
        def record_joint_command(speeds, **kwargs):
            joint_commands.append(list(speeds))
            return 0

        node.arm.vc_set_joint_velocity = record_joint_command

        # Deadman + Options captures the RCM point.
        node.input.inject(btn_rb=True, options_edge=True)
        node._control_loop_inner()

        assert node.sm.state == TeleopState.RCM_ACTIVE

        # Skip only the intentional mode-switch delay in this test.
        node._v7_constrained_settle_until_s = 0.0

        # Right-stick input should produce a mode-4 joint command.
        node.input.inject(options_edge=False, rx=0.5)
        node._control_loop_inner()

        assert node.arm.mode == 4
        assert joint_commands
        assert any(abs(v) > 0.0 for v in joint_commands[-1])

        # Options exits RCM and sends joint zero.
        node.input.inject(options_edge=True, rx=0.0)
        node._control_loop_inner()

        assert node.sm.state == TeleopState.FREE_TELEOP
        assert joint_commands[-1] == [0.0] * 7

        # Enter RCM again.
        node.input.inject(
            btn_rb=True,
            options_edge=True,
            rx=0.0,
        )
        node._control_loop_inner()

        assert node.sm.state == TeleopState.RCM_ACTIVE

        node._v7_constrained_settle_until_s = 0.0

        # Generate another nonzero joint command.
        node.input.inject(options_edge=False, rx=0.5)
        node._control_loop_inner()

        assert any(abs(v) > 0.0 for v in joint_commands[-1])

        # Releasing the deadman must stop RCM and enter IDLE.
        node.input.inject(btn_rb=False, rx=0.0)
        node._control_loop_inner()

        assert node.sm.state == TeleopState.IDLE
        assert joint_commands[-1] == [0.0] * 7

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()