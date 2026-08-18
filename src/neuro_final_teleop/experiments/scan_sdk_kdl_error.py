"""Compare SDK and KDL FK over virtual joint configurations.

The script requests mathematical FK results only.
It sends no robot motion command.
"""

import csv
from pathlib import Path

import numpy as np

from xarm.wrapper import XArmAPI

from neuro_final_teleop.control.kinematics import KDLKinModel
from neuro_final_teleop.control.math_utils import (
    rpy_deg_to_rotmat,
)

from scipy.spatial.transform import Rotation

ROBOT_IP = "192.168.1.243"
OUTPUT_CSV = Path("rcm_logs/sdk_kdl_fk_scan.csv")
JOINT_CHANGES_DEG = (5.0, 10.0, 20.0)
RANDOM_SAMPLE_COUNT = 100
RANDOM_CHANGE_DEG = 15.0
RANDOM_SEED = 7


def make_configurations(base_joints_deg):
    """Create baseline and single-joint virtual poses."""

    configurations = [
        ("baseline", np.asarray(base_joints_deg, dtype=float))
    ]

    for joint_index in range(7):
        for size in JOINT_CHANGES_DEG:
            for direction, sign in (
                ("plus", 1.0),
                ("minus", -1.0),
            ):
                joints = np.asarray(
                    base_joints_deg,
                    dtype=float,
                ).copy()
                joints[joint_index] += sign * size

                label = (
                    f"j{joint_index + 1}_"
                    f"{direction}_{size:g}"
                )
                configurations.append((label, joints))

    rng = np.random.default_rng(RANDOM_SEED)

    for index in range(RANDOM_SAMPLE_COUNT):
        change = rng.uniform(
            -RANDOM_CHANGE_DEG,
            RANDOM_CHANGE_DEG,
            size=7,
        )
        joints = (
            np.asarray(base_joints_deg, dtype=float)
            + change
        )

        label = f"combined_{index + 1:03d}"
        configurations.append((label, joints))

    return configurations


def compare_pose(arm, kin, joints_deg, tcp_offset):
    """Compare SDK and KDL FK for one virtual pose."""

    code, sdk_pose = arm.get_forward_kinematics(
        joints_deg.tolist(),
        input_is_radian=False,
        return_is_radian=False,
    )

    if code != 0:
        raise RuntimeError(f"SDK FK failed: code={code}")

    joints_rad = np.radians(joints_deg)
    tcp_translation_m = (
        np.asarray(tcp_offset[:3], dtype=float) * 0.001
    )

    kdl_position_m, kdl_flange_rotation = kin.fk_tool(
        joints_rad.tolist(),
        tcp_translation_m.tolist(),
    )

    sdk_position_mm = np.asarray(sdk_pose[:3], dtype=float)
    kdl_position_mm = np.asarray(kdl_position_m) * 1000.0

    sdk_rotation = rpy_deg_to_rotmat(*sdk_pose[3:6])
    tcp_rotation = rpy_deg_to_rotmat(*tcp_offset[3:6])
    kdl_rotation = kdl_flange_rotation @ tcp_rotation

    position_error_mm = sdk_position_mm - kdl_position_mm

    relative_rotation = sdk_rotation @ kdl_rotation.T

    rotation_error_vector_deg = np.degrees(
        Rotation.from_matrix(
            relative_rotation
        ).as_rotvec()
    )
    rotation_error_deg = float(
        np.linalg.norm(rotation_error_vector_deg)
    )
    return {
        "position_error_mm": position_error_mm,
        "position_error_norm_mm": float(
            np.linalg.norm(position_error_mm)
        ),
        "rotation_error_vector_deg":
            rotation_error_vector_deg,
        "rotation_error_deg": rotation_error_deg,
    }


def save_results(results):
    """Save the virtual FK comparison results."""

    OUTPUT_CSV.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    header = (
        ["label"]
        + [f"q{i}_deg" for i in range(1, 8)]
        + [
            "error_x_mm",
            "error_y_mm",
            "error_z_mm",
            "position_error_norm_mm",
            "rotation_x_deg",
            "rotation_y_deg",
            "rotation_z_deg",
            "rotation_error_deg",
        ]
    )

    with OUTPUT_CSV.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(header)

        for item in results:
            error = item["position_error_mm"]
            rotation = item["rotation_error_vector_deg"]

            writer.writerow(
                [item["label"]]
                + list(item["joints_deg"])
                + list(error)
                + [
                    item["position_error_norm_mm"],
                ]
                + list(rotation)
                + [
                    item["rotation_error_deg"],
                ]
            )

    print(f"\nSaved: {OUTPUT_CSV}")


def main():
    arm = XArmAPI(ROBOT_IP)
    kin = KDLKinModel()

    try:
        if not arm.connected:
            raise RuntimeError("Cannot connect to xArm")

        code, base_joints = arm.get_servo_angle(
            is_radian=False
        )

        if code != 0:
            raise RuntimeError(
                f"Joint read failed: code={code}"
            )

        base_joints = np.asarray(
            base_joints[:7],
            dtype=float,
        )
        tcp_offset = list(arm.tcp_offset[:6])

        configurations = make_configurations(base_joints)
        results = []

        print("Virtual SDK/KDL FK scan")

        for label, joints_deg in configurations:
            result = compare_pose(
                arm,
                kin,
                joints_deg,
                tcp_offset,
            )
            result["label"] = label
            result["joints_deg"] = joints_deg
            results.append(result)

            print(
                f"  {label:<12} "
                f"position={result['position_error_norm_mm']:.3f} mm  "
                f"rotation={result['rotation_error_deg']:.3f} deg"
            )

    finally:
        arm.disconnect()

    save_results(results)


if __name__ == "__main__":
    main()