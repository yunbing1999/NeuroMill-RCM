import math

import numpy as np
import pytest

from neuro_final_teleop.control.rcm_controller import (
    RCMConfig,
    RCMController,
)


class FakeKinematics:
    """Simple deterministic kinematics used only for controller tests."""

    def fk_tool(self, q_rad, tcp_offset_m=None):
        q = np.asarray(q_rad[:7], dtype=float)

        position = [
            float(q[0]),
            float(q[1]),
            float(q[2]),
        ]

        rotation = np.eye(3, dtype=float)
        return position, rotation

    def tool_jacobian(self, q_rad, tcp_offset_m=None):
        jacobian = np.zeros((6, 7), dtype=float)

        # Joints 1–3 produce tool translation.
        jacobian[0, 0] = 1.0
        jacobian[1, 1] = 1.0
        jacobian[2, 2] = 1.0

        # Joints 4–6 produce tool rotation.
        jacobian[3, 3] = 1.0
        jacobian[4, 4] = 1.0
        jacobian[5, 5] = 1.0

        return jacobian


def make_controller():
    config = RCMConfig(
        correction_gain_s=12.0,
        max_correction_mm_s=10.0,
        damping=0.001,
        qdot_limit_rad_s=1.0,
        qddot_limit_rad_s2=100.0,
        shaft_axis_sign=1.0,
    )

    return RCMController(
        FakeKinematics(),
        config,
    )


def test_initial_state_is_inactive():
    controller = make_controller()

    assert controller.active is False
    assert controller.entry_point_m is None
    assert controller.captured_axis is None


def test_capture_rejects_short_joint_vector():
    controller = make_controller()

    assert controller.capture(
        [0.0, 0.0],
        [0.0, 0.0, 0.0],
    ) is False

    assert controller.active is False


def test_capture_records_tip_and_axis():
    controller = make_controller()

    q = [
        0.10,
        -0.20,
        0.30,
        0.0,
        0.0,
        0.0,
        0.0,
    ]

    captured = controller.capture(
        q,
        [0.0, 0.0, 0.0],
    )

    assert captured is True
    assert controller.active is True

    assert controller.entry_point_m == pytest.approx(
        [0.10, -0.20, 0.30]
    )

    assert controller.captured_axis == pytest.approx(
        [0.0, 0.0, 1.0]
    )


def test_reset_clears_capture():
    controller = make_controller()

    assert controller.capture(
        [0.0] * 7,
        [0.0, 0.0, 0.0],
    )

    controller.reset()

    assert controller.active is False
    assert controller.entry_point_m is None
    assert controller.captured_axis is None


def test_solve_before_capture_raises():
    controller = make_controller()

    with pytest.raises(
        RuntimeError,
        match="before capture",
    ):
        controller.solve(
            q_rad=[0.0] * 7,
            desired_angular_rad_s=[0.0, 0.0, 0.0],
            desired_insertion_m_s=0.0,
            tcp_offset_m=[0.0, 0.0, 0.0],
            dt_s=0.01,
        )


def test_zero_command_at_capture_is_near_zero():
    controller = make_controller()
    q = [0.0] * 7

    assert controller.capture(
        q,
        [0.0, 0.0, 0.0],
    )

    result = controller.solve(
        q_rad=q,
        desired_angular_rad_s=[0.0, 0.0, 0.0],
        desired_insertion_m_s=0.0,
        tcp_offset_m=[0.0, 0.0, 0.0],
        dt_s=0.01,
    )

    assert result.lateral_error_mm == pytest.approx(
        0.0,
        abs=1e-9,
    )

    assert result.qdot_rad_s == pytest.approx(
        [0.0] * 7,
        abs=1e-7,
    )


def test_lateral_error_generates_correction():
    controller = make_controller()

    capture_q = [0.0] * 7

    assert controller.capture(
        capture_q,
        [0.0, 0.0, 0.0],
    )

    # Move the modeled tip 1 mm in base X after capture.
    displaced_q = [
        0.001,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]

    result = controller.solve(
        q_rad=displaced_q,
        desired_angular_rad_s=[0.0, 0.0, 0.0],
        desired_insertion_m_s=0.0,
        tcp_offset_m=[0.0, 0.0, 0.0],
        dt_s=0.01,
    )

    assert result.lateral_error_mm == pytest.approx(
        1.0,
        abs=1e-6,
    )

    assert result.entry_point_mm == pytest.approx(
        [0.0, 0.0, 0.0]
    )
    assert result.shaft_point_mm == pytest.approx(
        [1.0, 0.0, 0.0]
    )
    assert result.lateral_error_vector_mm == pytest.approx(
        [-1.0, 0.0, 0.0]
    )

    # Correction should command joint 1 toward negative X.
    assert result.qdot_rad_s[0] < 0.0


def test_axial_displacement_is_not_lateral_error():
    controller = make_controller()

    assert controller.capture(
        [0.0] * 7,
        [0.0, 0.0, 0.0],
    )

    # Tool Z is base Z in FakeKinematics. Moving along Z must not
    # produce lateral RCM error.
    displaced_q = [
        0.0,
        0.0,
        0.020,
        0.0,
        0.0,
        0.0,
        0.0,
    ]

    result = controller.solve(
        q_rad=displaced_q,
        desired_angular_rad_s=[0.0, 0.0, 0.0],
        desired_insertion_m_s=0.0,
        tcp_offset_m=[0.0, 0.0, 0.0],
        dt_s=0.01,
    )

    assert result.lateral_error_mm == pytest.approx(
        0.0,
        abs=1e-9,
    )

    assert result.insertion_depth_mm == pytest.approx(
        20.0,
        abs=1e-9,
    )


def test_insertion_stops_at_positive_depth_limit():
    controller = make_controller()
    controller.cfg.max_insertion_depth_mm = 20.0

    assert controller.capture(
        [0.0] * 7,
        [0.0] * 3,
    )

    q = [0.0] * 7
    q[2] = 0.020

    result = controller.solve(
        q_rad=q,
        desired_angular_rad_s=[0.0] * 3,
        desired_insertion_m_s=0.005,
        tcp_offset_m=[0.0] * 3,
        dt_s=0.01,
    )

    assert result.insertion_depth_mm == pytest.approx(20.0)
    assert result.insertion_mm_s == pytest.approx(0.0)


def test_insertion_slows_near_positive_limit():
    controller = make_controller()

    assert controller.capture([0.0] * 7, [0.0] * 3)

    q = [0.0] * 7
    q[2] = 0.018

    result = controller.solve(
        q_rad=q,
        desired_angular_rad_s=[0.0] * 3,
        desired_insertion_m_s=0.005,
        tcp_offset_m=[0.0] * 3,
        dt_s=0.01,
    )

    assert result.insertion_mm_s == pytest.approx(
        2.0,
        abs=0.1,
    )
    assert result.limited is True


def test_repeated_insertion_stays_inside_positive_limit():
    controller = make_controller()
    controller.cfg.qddot_limit_rad_s2 = 0.05

    q = np.zeros(7)
    dt = 0.01

    assert controller.capture(q.tolist(), [0.0] * 3)

    max_depth_mm = 0.0

    for _ in range(1000):
        result = controller.solve(
            q_rad=q.tolist(),
            desired_angular_rad_s=[0.0] * 3,
            desired_insertion_m_s=0.005,
            tcp_offset_m=[0.0] * 3,
            dt_s=dt,
        )

        q += np.asarray(result.qdot_rad_s) * dt
        max_depth_mm = max(max_depth_mm, q[2] * 1000.0)

    assert max_depth_mm <= 20.001
    assert q[2] * 1000.0 > 19.9


def test_repeated_withdrawal_stays_inside_negative_limit():
    controller = make_controller()
    controller.cfg.qddot_limit_rad_s2 = 0.05

    q = np.zeros(7)
    dt = 0.01

    assert controller.capture(q.tolist(), [0.0] * 3)

    min_depth_mm = 0.0

    for _ in range(1000):
        result = controller.solve(
            q_rad=q.tolist(),
            desired_angular_rad_s=[0.0] * 3,
            desired_insertion_m_s=-0.005,
            tcp_offset_m=[0.0] * 3,
            dt_s=dt,
        )

        q += np.asarray(result.qdot_rad_s) * dt
        min_depth_mm = min(min_depth_mm, q[2] * 1000.0)

    assert min_depth_mm >= -10.001
    assert q[2] * 1000.0 < -9.9


def test_withdrawal_is_allowed_at_positive_limit():
    controller = make_controller()

    assert controller.capture(
        [0.0] * 7,
        [0.0] * 3,
    )

    q = [0.0] * 7
    q[2] = 0.020

    result = controller.solve(
        q_rad=q,
        desired_angular_rad_s=[0.0] * 3,
        desired_insertion_m_s=-0.005,
        tcp_offset_m=[0.0] * 3,
        dt_s=0.01,
    )

    assert result.insertion_mm_s == pytest.approx(
        -5.0,
        abs=0.1,
    )
    assert result.limited is False


def test_withdrawal_slows_near_negative_limit():
    controller = make_controller()

    assert controller.capture([0.0] * 7, [0.0] * 3)

    q = [0.0] * 7
    q[2] = -0.008

    result = controller.solve(
        q_rad=q,
        desired_angular_rad_s=[0.0] * 3,
        desired_insertion_m_s=-0.005,
        tcp_offset_m=[0.0] * 3,
        dt_s=0.01,
    )

    assert result.insertion_mm_s == pytest.approx(
        -2.0,
        abs=0.1,
    )
    assert result.limited is True


def test_withdrawal_stops_at_negative_depth_limit():
    controller = make_controller()
    controller.cfg.max_withdrawal_depth_mm = 10.0

    assert controller.capture(
        [0.0] * 7,
        [0.0] * 3,
    )

    q = [0.0] * 7
    q[2] = -0.010

    result = controller.solve(
        q_rad=q,
        desired_angular_rad_s=[0.0] * 3,
        desired_insertion_m_s=-0.005,
        tcp_offset_m=[0.0] * 3,
        dt_s=0.01,
    )

    assert result.insertion_depth_mm == pytest.approx(-10.0)
    assert result.insertion_mm_s == pytest.approx(0.0)
    assert result.limited is True


def test_insertion_is_allowed_at_negative_limit():
    controller = make_controller()

    assert controller.capture(
        [0.0] * 7,
        [0.0] * 3,
    )

    q = [0.0] * 7
    q[2] = -0.010

    result = controller.solve(
        q_rad=q,
        desired_angular_rad_s=[0.0] * 3,
        desired_insertion_m_s=0.005,
        tcp_offset_m=[0.0] * 3,
        dt_s=0.01,
    )

    assert result.insertion_mm_s == pytest.approx(
        5.0,
        abs=0.1,
    )
    assert result.limited is False


def test_insertion_command_produces_axial_velocity():
    controller = make_controller()

    q = [0.0] * 7

    assert controller.capture(
        q,
        [0.0, 0.0, 0.0],
    )

    result = controller.solve(
        q_rad=q,
        desired_angular_rad_s=[0.0, 0.0, 0.0],
        desired_insertion_m_s=0.005,
        tcp_offset_m=[0.0, 0.0, 0.0],
        dt_s=0.01,
    )

    assert result.insertion_mm_s == pytest.approx(
        5.0,
        abs=0.1,
    )

    assert result.lateral_error_mm == pytest.approx(
        0.0,
        abs=1e-9,
    )


def test_angular_command_produces_rotation():
    controller = make_controller()

    q = [0.0] * 7

    assert controller.capture(
        q,
        [0.0, 0.0, 0.0],
    )

    desired_speed = math.radians(5.0)

    result = controller.solve(
        q_rad=q,
        desired_angular_rad_s=[
            desired_speed,
            0.0,
            0.0,
        ],
        desired_insertion_m_s=0.0,
        tcp_offset_m=[0.0, 0.0, 0.0],
        dt_s=0.01,
    )

    assert result.angular_rad_s[0] == pytest.approx(
        desired_speed,
        abs=1e-3,
    )

    assert result.lateral_error_mm == pytest.approx(
        0.0,
        abs=1e-9,
    )


def test_combined_operator_task_tracks_insertion_and_rotation():
    controller = make_controller()
    q = [0.0] * 7

    assert controller.capture(
        q,
        [0.0, 0.0, 0.0],
    )

    desired_speed = math.radians(5.0)
    result = controller.solve(
        q_rad=q,
        desired_angular_rad_s=[desired_speed, 0.0, 0.0],
        desired_insertion_m_s=0.005,
        tcp_offset_m=[0.0, 0.0, 0.0],
        dt_s=0.01,
    )

    assert result.insertion_mm_s == pytest.approx(
        5.0,
        abs=0.1,
    )
    assert result.angular_rad_s[0] == pytest.approx(
        desired_speed,
        abs=1e-3,
    )
    assert result.lateral_error_mm == pytest.approx(
        0.0,
        abs=1e-9,
    )


def test_combined_operator_task_preserves_rcm_correction():
    correction_only_controller = make_controller()
    combined_controller = make_controller()

    capture_q = [0.0] * 7

    assert correction_only_controller.capture(
        capture_q,
        [0.0, 0.0, 0.0],
    )
    assert combined_controller.capture(
        capture_q,
        [0.0, 0.0, 0.0],
    )

    # Create a 1 mm lateral RCM error in base X.
    displaced_q = [
        0.001,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]

    correction_only = correction_only_controller.solve(
        q_rad=displaced_q,
        desired_angular_rad_s=[0.0, 0.0, 0.0],
        desired_insertion_m_s=0.0,
        tcp_offset_m=[0.0, 0.0, 0.0],
        dt_s=0.01,
    )

    desired_speed = math.radians(5.0)

    combined = combined_controller.solve(
        q_rad=displaced_q,
        desired_angular_rad_s=[
            desired_speed,
            0.0,
            0.0,
        ],
        desired_insertion_m_s=0.005,
        tcp_offset_m=[0.0, 0.0, 0.0],
        dt_s=0.01,
    )

    assert combined.lateral_error_mm == pytest.approx(
        1.0,
        abs=1e-6,
    )

    # Adding P2 must not change the P1 lateral correction command.
    assert combined.qdot_rad_s[:2] == pytest.approx(
        correction_only.qdot_rad_s[:2],
        abs=1e-9,
    )

    assert combined.insertion_mm_s == pytest.approx(
        5.0,
        abs=0.1,
    )
    assert combined.angular_rad_s[0] == pytest.approx(
        desired_speed,
        abs=1e-3,
    )


def test_joint_velocity_limit():
    controller = make_controller()
    controller.cfg.qdot_limit_rad_s = 0.10

    q = [0.0] * 7

    assert controller.capture(
        q,
        [0.0, 0.0, 0.0],
    )

    result = controller.solve(
        q_rad=q,
        desired_angular_rad_s=[1.0, 0.0, 0.0],
        desired_insertion_m_s=0.0,
        tcp_offset_m=[0.0, 0.0, 0.0],
        dt_s=0.01,
    )

    assert max(
        abs(value)
        for value in result.qdot_rad_s
    ) <= 0.100001

    assert result.limited is True
