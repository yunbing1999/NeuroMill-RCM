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
