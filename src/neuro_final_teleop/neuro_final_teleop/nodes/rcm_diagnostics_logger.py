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

        self._fields = (
            "time_s",
            "lateral_error_mm",
            "desired_angular_speed_rad_s",
            "achieved_angular_speed_rad_s",
            "angular_error_rad_s",
            "requested_insertion_mm_s",
            "target_insertion_mm_s",
            "achieved_insertion_mm_s",
            "insertion_depth_mm",
            "max_joint_speed_rad_s",
            "travel_limited",
            "joint_velocity_limited",
            "joint_acceleration_limited",
        )
        header = list(self._fields)
        self._writer.writerow(header)
        self._file.flush()

        # Values retained only for the final summary.
        self._errors = []
        self._joint_speeds = []
        self._angular_errors = []
        self._insertion_errors = []
        self._travel_limited_count = 0
        self._velocity_limited_count = 0
        self._acceleration_limited_count = 0

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

            values = [data[field] for field in self._fields]
            values[-3:] = [int(value) for value in values[-3:]]
            self._writer.writerow(values)
            self._file.flush()

            error_mm = float(data["lateral_error_mm"])
            angular_error = float(data["angular_error_rad_s"])
            insertion_error = abs(
                float(data["target_insertion_mm_s"])
                - float(data["achieved_insertion_mm_s"])
            )
            max_joint_speed = float(data["max_joint_speed_rad_s"])

            self._errors.append(error_mm)
            self._joint_speeds.append(max_joint_speed)
            self._angular_errors.append(angular_error)
            self._insertion_errors.append(insertion_error)
            self._travel_limited_count += int(data["travel_limited"])
            self._velocity_limited_count += int(data["joint_velocity_limited"])
            self._acceleration_limited_count += int(data["joint_acceleration_limited"])

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
                p95_error = float(np.percentile(errors, 95))
                max_joint_speed = max(self._joint_speeds)
                mean_angular_error = float(
                    np.mean(self._angular_errors)
                )
                mean_insertion_error = float(
                    np.mean(self._insertion_errors)
                )

                print(
                    "\nRCM recording summary"
                    f"\n  Samples:              {len(errors)}"
                    f"\n  Mean RCM error:       {mean_error:.6f} mm"
                    f"\n  Maximum RCM error:    {max_error:.6f} mm"
                    f"\n  RMS RCM error:        {rms_error:.6f} mm"
                    f"\n  95th percentile:      {p95_error:.6f} mm"
                    f"\n  Maximum joint speed:  {max_joint_speed:.6f} rad/s"
                    f"\n  Mean angular error:   {mean_angular_error:.6f} rad/s"
                    f"\n  Mean insertion error: {mean_insertion_error:.6f} mm/s"
                    f"\n  Travel limited:       {self._travel_limited_count}/{len(errors)}"
                    f"\n  Velocity limited:     {self._velocity_limited_count}/{len(errors)}"
                    f"\n  Acceleration limited: {self._acceleration_limited_count}/{len(errors)}"
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
