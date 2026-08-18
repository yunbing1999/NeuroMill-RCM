"""Run the SDK/KDL scan with robot-specific kinematics."""

from pathlib import Path

import yaml

import neuro_final_teleop.control.kinematics as kinematics
import scan_sdk_kdl_error as scan


KINEMATICS_YAML = Path(
    "src/external/xarm_description/config/kinematics/"
    "user/xarm7_kinematics_neuromill_check.yaml"
)

OUTPUT_CSV = Path(
    "rcm_logs/sdk_kdl_fk_scan_calibrated.csv"
)


def load_joint_origins():
    """Load the robot-specific joint origin transforms."""

    with KINEMATICS_YAML.open() as file:
        data = yaml.safe_load(file)["kinematics"]

    fields = (
        "x",
        "y",
        "z",
        "roll",
        "pitch",
        "yaw",
    )
    origins = []

    for index in range(1, 8):
        joint = data[f"joint{index}"]
        origins.append(
            tuple(float(joint[field]) for field in fields)
        )

    return origins


def main():
    joint_origins = load_joint_origins()

    # Process-local replacement for this experiment only.
    kinematics._JOINT_ORIGINS = joint_origins
    scan.OUTPUT_CSV = OUTPUT_CSV

    print(f"Using calibrated kinematics: {KINEMATICS_YAML}")
    scan.main()


if __name__ == "__main__":
    main()