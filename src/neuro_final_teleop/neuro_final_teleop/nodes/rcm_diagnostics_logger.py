"""Record RCM diagnostics to CSV without controlling the robot."""

import csv
import json
import math
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RCMDiagnosticsLogger(Node):
    """Subscribe to RCM diagnostics and save each sample."""

    def __init__(self):
        super().__init__("rcm_diagnostics_logger")

        self.declare_parameter("output_dir", "rcm_logs")
        output_dir = Path(
            self.get_parameter("output_dir").value
        ).expanduser()
        output_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_path = output_dir / f"rcm_{timestamp}.csv"

        self._file = self.output_path.open("w", newline="")
        self._writer = csv.writer(self._file)

        header = (
            ["time_s"]
            + [f"q{i}_rad" for i in range(1, 8)]
            + [f"qdot{i}_rad_s" for i in range(1, 8)]
            + ["entry_x_mm", "entry_y_mm", "entry_z_mm"]
            + ["shaft_x_mm", "shaft_y_mm", "shaft_z_mm"]
            + ["error_x_mm", "error_y_mm", "error_z_mm"]
            + ["lateral_error_mm"]
            + ["desired_wx_rad_s", "desired_wy_rad_s", "desired_wz_rad_s"]
            + ["achieved_wx_rad_s", "achieved_wy_rad_s", "achieved_wz_rad_s"]
            + ["insertion_depth_mm", "insertion_mm_s", "limited", "mode", "state"]
        )
        self._writer.writerow(header)
        self._file.flush()

        # Values retained only for the final summary.
        self._errors = []
        self._joint_speeds = []
        self._angular_errors = []
        self._limited_count = 0

        self._subscription = self.create_subscription(
            String,
            "/neuro_final/rcm_diagnostics",
            self._on_diagnostics,
            10,
        )

        self.get_logger().info(f"Recording RCM data: {self.output_path}")

    def _on_diagnostics(self, message):
        """Parse one diagnostics message and save it."""

        try:
            data = json.loads(message.data)

            joints = list(data["joints_rad"])
            qdot = list(data["qdot_rad_s"])
            entry = list(data["entry_mm"])
            shaft = list(data["shaft_mm"])
            error_vector = list(data["error_vector_mm"])
            desired_w = list(data["desired_w_rad_s"])
            achieved_w = list(data["achieved_w_rad_s"])

            expected_lengths = (
                (joints, 7),
                (qdot, 7),
                (entry, 3),
                (shaft, 3),
                (error_vector, 3),
                (desired_w, 3),
                (achieved_w, 3),
            )
            if any(len(values) != size for values, size in expected_lengths):
                raise ValueError("Unexpected diagnostics vector length")

            error_mm = float(data["error_mm"])
            angular_error = float(
                np.linalg.norm(
                    np.asarray(desired_w) - np.asarray(achieved_w)
                )
            )
            max_joint_speed = float(np.max(np.abs(qdot)))

            self._writer.writerow(
                [data["time_s"]]
                + joints
                + qdot
                + entry
                + shaft
                + error_vector
                + [error_mm]
                + desired_w
                + achieved_w
                + [
                    data["insertion_depth_mm"],
                    data["insertion_mm_s"],
                    int(data["limited"]),
                    data["mode"],
                    data["state"],
                ]
            )
            self._file.flush()

            self._errors.append(error_mm)
            self._joint_speeds.append(max_joint_speed)
            self._angular_errors.append(angular_error)
            self._limited_count += int(data["limited"])

        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(
                f"Invalid RCM diagnostics message: {exc}"
            )

    
    def destroy_node(self):
        """Print the summary and safely close the CSV file."""

        if not self._file.closed:
            if self._errors:
                errors = np.asarray(self._errors, dtype=float)

                mean_error = float(np.mean(errors))
                max_error = float(np.max(errors))
                rms_error = float(math.sqrt(np.mean(errors**2)))
                max_joint_speed = max(self._joint_speeds)
                mean_angular_error = float(
                    np.mean(self._angular_errors)
                )

                print(
                    "\nRCM recording summary"
                    f"\n  Samples:              {len(errors)}"
                    f"\n  Mean RCM error:       {mean_error:.6f} mm"
                    f"\n  Maximum RCM error:    {max_error:.6f} mm"
                    f"\n  RMS RCM error:        {rms_error:.6f} mm"
                    f"\n  Maximum joint speed:  {max_joint_speed:.6f} rad/s"
                    f"\n  Mean angular error:   {mean_angular_error:.6f} rad/s"
                    f"\n  Limited samples:      {self._limited_count}/{len(errors)}"
                )
            else:
                print("\nNo RCM diagnostics samples were received")

            self._file.flush()
            self._file.close()
            print(f"Saved CSV: {self.output_path}")

        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RCMDiagnosticsLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()        