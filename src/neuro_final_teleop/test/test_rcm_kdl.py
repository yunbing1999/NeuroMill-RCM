import math

import numpy as np
import PyKDL
import pytest

from neuro_final_teleop.control.kinematics import KDLKinModel
from neuro_final_teleop.control.math_utils import (
    rotmat_to_rpy_deg,
    rpy_deg_to_rotmat,
)
from neuro_final_teleop.control.rcm_controller import (
    RCMConfig,
    RCMController,
)


def test_fk_tool_identity_rotation_preserves_old_behavior():
    kin = KDLKinModel()
    q = [0.10, -0.40, 0.20, 0.80, -0.10, 0.40, 0.30]
    offset = [-0.03207, -0.01112, 0.24586]

    old_position, old_rotation = kin.fk_tool(q, offset)
    new_position, new_rotation = kin.fk_tool(q, offset, np.eye(3))

    assert new_position == old_position
    assert np.array_equal(new_rotation, old_rotation)


def test_fk_tool_applies_tcp_rotation_without_moving_tcp():
    kin = KDLKinModel()
    q = [0.10, -0.40, 0.20, 0.80, -0.10, 0.40, 0.30]
    offset = [-0.03207, -0.01112, 0.24586]
    tcp_rotation = rpy_deg_to_rotmat(4.47, 2.77, 7.13)

    flange_position, flange_rotation = kin.fk_flange(q)
    identity_position, _ = kin.fk_tool(q, offset)
    tool_position, tool_rotation = kin.fk_tool(
        q,
        offset,
        tcp_rotation,
    )

    assert flange_position != identity_position
    assert tool_position == identity_position
    assert tool_rotation == pytest.approx(flange_rotation @ tcp_rotation)

    flange_z = flange_rotation[:, 2]
    tool_z = tool_rotation[:, 2]
    angle_deg = math.degrees(
        math.acos(float(np.clip(flange_z @ tool_z, -1.0, 1.0)))
    )

    assert angle_deg == pytest.approx(5.25, abs=0.05)
    assert float(flange_z @ tool_z) > 0.99


def test_rpy_rotation_matches_pykdl():
    rpy_deg = (4.47, 2.77, 7.13)
    actual = rpy_deg_to_rotmat(*rpy_deg)
    kdl_rotation = PyKDL.Rotation.RPY(
        *[math.radians(value) for value in rpy_deg]
    )
    expected = np.array(
        [[kdl_rotation[row, col] for col in range(3)] for row in range(3)]
    )

    assert actual == pytest.approx(expected)


def test_sdk_orientation_validation_uses_rotated_tcp_frame():
    kin = KDLKinModel()
    q = [0.10, -0.40, 0.20, 0.80, -0.10, 0.40, 0.30]
    offset_mm_deg = [-32.07, -11.12, 245.86, 4.47, 2.77, 7.13]
    offset_m = [value * 0.001 for value in offset_mm_deg[:3]]
    tcp_rotation = rpy_deg_to_rotmat(*offset_mm_deg[3:6])
    position_m, rotation = kin.fk_tool(q, offset_m, tcp_rotation)
    sdk_pose = [
        *[value * 1000.0 for value in position_m],
        *rotmat_to_rpy_deg(rotation),
    ]

    result = kin.validate_against_sdk(
        sdk_pose,
        q,
        tcp_offset_mm_deg=offset_mm_deg,
        tcp_rotation=tcp_rotation,
    )

    assert result.valid is True
    assert result.position_error_mm == pytest.approx(0.0, abs=1e-10)
    assert result.orientation_error_deg == pytest.approx(0.0, abs=1e-6)


def test_rcm_with_real_kdl_model():
    kin = KDLKinModel()

    controller = RCMController(
        kin,
        RCMConfig(
            correction_gain_s=12.0,
            max_correction_mm_s=10.0,
            damping=0.01,
            qdot_limit_rad_s=0.30,
            qddot_limit_rad_s2=10.0,
            shaft_axis_sign=1.0,
        ),
    )

    # Nonzero, bent-arm configuration to avoid the all-zero singular pose.
    q = np.array(
        [
            0.0,
            -0.40,
            0.0,
            0.80,
            0.0,
            0.40,
            0.0,
        ],
        dtype=float,
    )

    # Example 150 mm instrument/TCP offset along tool Z.
    tcp_offset_m = [0.0, 0.0, 0.150]

    assert controller.capture(
        q.tolist(),
        tcp_offset_m,
    )

    assert controller.entry_point_m is not None

    dt_s = 0.01
    desired_angular = [
        0.0,
        math.radians(1.0),
        0.0,
    ]

    maximum_rcm_error_mm = 0.0
    maximum_joint_speed = 0.0
    maximum_angular_speed = 0.0

    # Simulate two seconds of resolved-rate control.
    for _ in range(200):
        result = controller.solve(
            q_rad=q.tolist(),
            desired_angular_rad_s=desired_angular,
            desired_insertion_m_s=0.0,
            tcp_offset_m=tcp_offset_m,
            dt_s=dt_s,
        )

        qdot = np.asarray(
            result.qdot_rad_s,
            dtype=float,
        )

        assert qdot.shape == (7,)
        assert np.all(np.isfinite(qdot))

        maximum_rcm_error_mm = max(
            maximum_rcm_error_mm,
            result.lateral_error_mm,
        )

        maximum_joint_speed = max(
            maximum_joint_speed,
            float(np.max(np.abs(qdot))),
        )

        maximum_angular_speed = max(
            maximum_angular_speed,
            float(np.linalg.norm(result.angular_rad_s)),
        )

        # Euler integration for an offline numerical simulation.
        q = q + qdot * dt_s

    print(
        "\nKDL RCM result:"
        f" max_error={maximum_rcm_error_mm:.4f} mm,"
        f" max_qdot={maximum_joint_speed:.4f} rad/s,"
        f" max_angular={maximum_angular_speed:.4f} rad/s"
    )

    assert maximum_rcm_error_mm < 0.50
    assert maximum_joint_speed <= 0.300001
    assert maximum_angular_speed > 0.001
