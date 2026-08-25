import math
from neuro_final_teleop.control.state_machine import (
    StateMachine,
    TeleopState,
)
from neuro_final_teleop.input.dualsense import InputSnapshot
from neuro_final_teleop.motion_modes import MotionModesMixin
from types import SimpleNamespace


class DummyBase:
    """Provides the parent callback expected by MotionModesMixin."""

    def _on_state_exit(self, old, new, reason):
        pass


class FakeArm:
    """Minimal mode-4 xArm simulation."""

    def __init__(self):
        self.mode = 4
        self.state = 0
        self.error_code = 0
        self.warn_code = 0
        self.last_joint_cmd = None

    def get_servo_angle(self, is_radian=True):
        return 0, [0.0] * 7

    def vc_set_joint_velocity(self, speeds, **kwargs):
        self.last_joint_cmd = list(speeds)
        return 0

class FakeResettable:
    def reset(self):
        pass


class FakeRCMController:
    def __init__(self):
        self.entry_point_m = None
        self.captured_axis = None
        self.reset_called = False

    def capture(self, joints, tcp_offset_m):
        self.entry_point_m = [
            0.100,
            0.200,
            0.300,
        ]
        self.captured_axis = [
            0.0,
            0.0,
            1.0,
        ]
        return True

    @property
    def active(self):
        return self.entry_point_m is not None

    def reset(self):
        self.entry_point_m = None
        self.captured_axis = None
        self.reset_called = True


class FakeLogger:
    def info(self, message):
        pass


class RCMStateFlowHost(MotionModesMixin, DummyBase):
    def __init__(self):
        self.v7_rcm_enable = True
        self._constrained_modes_enabled = True
        self._v7_tip_lock_started_by_r3 = False

        self.v7_tip_lock_joint_ik_enable = False

        self._v7_joint_velocity_active = False
        self._current_mode = 4

        self._v7_rcm_joint_cmd_rad_s = [0.0] * 7
        self._v7_last_joint_cmd_rad_s = [0.0] * 7

        self.arm = FakeArm()
        self._v7_rcm_controller = FakeRCMController()
        self._v7_fixed_point_ik = FakeResettable()
        self.velocity_watchdog_s = 0.0



        self.sm = StateMachine(
            on_exit=self._on_state_exit,
        )

    def _joint_ft_scale(self):
        # Simulate 50% speed reduction from FT safety.
        return 0.5

    def _effective_tcp_translation_mm(self):
        return [0.0, 0.0, 150.0]

    def _pulse_haptic(
        self,
        strength,
        duration_ms,
        cooldown_s,
    ):
        pass

    def _warn_throttle(
        self,
        key,
        text,
        interval,
    ):
        raise AssertionError(
            f"Unexpected warning: {text}"
        )

    def get_logger(self):
        return FakeLogger()


def test_options_rcm_capture_and_exit_flow():
    host = RCMStateFlowHost()

    assert host.sm.state == TeleopState.IDLE

    assert host.sm.transition_to(
        TeleopState.FREE_TELEOP,
        "test_deadman",
    )

    # First Options press requests capture.
    host._handle_button_actions(
        InputSnapshot(
            options_edge=True,
        )
    )

    assert host.sm.state == TeleopState.RCM_CAPTURE

    # The control loop would call this on the following capture cycle.
    host._do_rcm_capture()

    assert host.sm.state == TeleopState.RCM_ACTIVE
    assert host._v7_rcm_controller.entry_point_m == [
        0.100,
        0.200,
        0.300,
    ]

    # Second Options press exits RCM.
    host._handle_button_actions(
        InputSnapshot(
            options_edge=True,
        )
    )

    assert host.sm.state == TeleopState.FREE_TELEOP
    assert host._v7_rcm_controller.reset_called is True
    assert host._v7_rcm_controller.entry_point_m is None
    assert host.arm.last_joint_cmd == [0.0] * 7


class FakeMotionRCM:
    active = True

    def solve(self, **kwargs):
        self.arguments = kwargs
        return SimpleNamespace(
            qdot_rad_s=[0.01] * 7,
            correction_mm_s=[0.0] * 3,
            angular_rad_s=[0.02, 0.0, 0.0],
            lateral_error_mm=0.0,
        )


class RCMRotationHost(MotionModesMixin):
    def __init__(self):
        self._v7_constrained_settle_until_s = 0.0
        self._v7_rcm_joint_cmd_rad_s = [0.0] * 7
        self._v7_rcm_controller = FakeMotionRCM()

        self.v7_rcm_max_angular_deg_s = 5.0
        self._v7_speed_scale = 1.0
        self.sigmoid_gain = 0.5
        self.dt = 0.01
        self._last_cmd_sent = [0.0] * 6

    def _effective_tip_pose_mm_deg(self, joints):
        # Identity tool orientation.
        return [0.0] * 6

    def _effective_tcp_translation_mm(self):
        return [0.0, 0.0, 150.0]

    def _warn_throttle(self, key, text, interval):
        raise AssertionError(text)

def test_right_stick_generates_rcm_joint_command():
    host = RCMRotationHost()

    cartesian_cmd = host._vel_rcm_joint_ik(
        InputSnapshot(rx=0.5, ry=0.0),
        scale=1.0,
        joints_rad=[0.0] * 7,
    )

    # RCM stores seven joint velocities, not a Cartesian command.
    assert cartesian_cmd == [0.0] * 6
    assert host._v7_rcm_joint_cmd_rad_s == [0.01] * 7

    arguments = host._v7_rcm_controller.arguments

    # Rotation phase must keep insertion disabled.
    assert arguments["desired_insertion_m_s"] == 0.0
    assert arguments["tcp_offset_m"] == [0.0, 0.0, 0.15]

    # Right-stick X should request angular motion.
    assert abs(arguments["desired_angular_rad_s"][0]) > 0.0

def test_diagonal_right_stick_respects_max_angular_speed():
    host = RCMRotationHost()

    host._vel_rcm_joint_ik(
        InputSnapshot(rx=1.0, ry=1.0),
        scale=1.0,
        joints_rad=[0.0] * 7,
    )

    desired_w = (
        host._v7_rcm_controller.arguments[
            "desired_angular_rad_s"
        ]
    )

    angular_norm = math.sqrt(
        sum(component * component for component in desired_w)
    )
    max_angular = math.radians(
        host.v7_rcm_max_angular_deg_s
    )

    assert math.isclose(
        angular_norm,
        max_angular,
        rel_tol=1e-9,
        abs_tol=1e-12,
    )

def test_rcm_joint_command_uses_mode4_and_ft_scale():
    host = RCMStateFlowHost()

    host.sm.transition_to(TeleopState.FREE_TELEOP, "test")
    host.sm.transition_to(TeleopState.RCM_CAPTURE, "test")
    host._do_rcm_capture()

    assert host.sm.state == TeleopState.RCM_ACTIVE

    host._v7_rcm_joint_cmd_rad_s = [0.20] * 7
    host._send_velocity([0.0] * 6)

    # FT safety reduces 0.20 rad/s to 0.10 rad/s.
    assert host.arm.last_joint_cmd == [0.10] * 7
    assert host._v7_joint_velocity_active is True