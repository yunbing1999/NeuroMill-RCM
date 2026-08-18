import math

import numpy as np

from neuro_final_teleop.control.kinematics import KDLKinModel
from neuro_final_teleop.control.rcm_controller import (
    RCMConfig,
    RCMController,
)


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
            nullspace_gain=0.0,
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