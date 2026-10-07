"""Record RCM diagnostics to CSV without controlling the robot."""

import csv
import json
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


BASIC_FIELDS = (
    "elapsed_s",
    "lateral_error_mm",
    "tcp_x_mm",
    "tcp_y_mm",
    "tcp_z_mm",
    "q1_deg",
    "q2_deg",
    "q3_deg",
    "q4_deg",
    "q5_deg",
    "q6_deg",
    "q7_deg",
)


class BasicRCMSessionWriter:
    """Write one basic CSV and metadata file per successful RCM capture."""

    def __init__(self, output_dir, monotonic_clock=None, wall_clock=None):
        self.output_dir = Path(output_dir).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self._monotonic_clock = monotonic_clock or time.monotonic
        self._wall_clock = wall_clock or datetime.now
        self._file = None
        self._writer = None
        self._start_monotonic = None
        self.output_path = None
        self.metadata_path = None

    @property
    def active(self):
        return self._file is not None and not self._file.closed

    def _new_paths(self):
        stamp = self._wall_clock().strftime("%Y%m%d_%H%M%S_%f")
        csv_path = self.output_dir / f"rcm_basic_{stamp}.csv"
        metadata_path = self.output_dir / f"rcm_basic_{stamp}_metadata.json"
        suffix = 1
        while csv_path.exists() or metadata_path.exists():
            csv_path = self.output_dir / f"rcm_basic_{stamp}_{suffix}.csv"
            metadata_path = (
                self.output_dir / f"rcm_basic_{stamp}_{suffix}_metadata.json"
            )
            suffix += 1
        return csv_path, metadata_path

    def start(self, metadata):
        """Close any previous session and create a fresh capture session."""
        self.stop()
        self.output_path, self.metadata_path = self._new_paths()
        self._file = self.output_path.open("w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(BASIC_FIELDS)
        self._file.flush()
        self._start_monotonic = None

        metadata_record = {
            "recorded_at_local": self._wall_clock().astimezone().isoformat(),
            "captured_rcm_point_base_mm": metadata.get(
                "captured_rcm_point_base_mm", []
            ),
            "tcp_offset_mm_deg": metadata.get("tcp_offset_mm_deg", []),
            "effective_tcp_translation_mm": metadata.get(
                "effective_tcp_translation_mm", []
            ),
            "model_configuration": metadata.get("model_configuration", {}),
            "coordinate_frames": {
                "tcp_xyz": "robot Base",
                "captured_rcm_point": "robot Base",
            },
            "units": {
                "elapsed_s": "s",
                "lateral_error_mm": "mm",
                "tcp_xyz": "mm",
                "joint_angles": "deg",
            },
            "data_sources": {
                "joint_angles": "xArm joint encoder feedback",
                "tcp_xyz": (
                    "calibrated KDL forward kinematics evaluated with the "
                    "same feedback joint sample"
                ),
                "lateral_error_mm": "RCM controller KDL model result",
            },
        }
        self.metadata_path.write_text(
            json.dumps(metadata_record, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        return self.output_path, self.metadata_path

    def write_sample(self, data):
        """Write one fixed-decimal sample if an RCM session is active."""
        if not self.active:
            return False

        tcp_mm = data["tcp_position_mm"]
        joints_deg = data["joints_deg"]
        if len(tcp_mm) != 3 or len(joints_deg) != 7:
            raise ValueError("basic RCM sample requires 3 TCP and 7 joint values")

        now = self._monotonic_clock()
        if self._start_monotonic is None:
            self._start_monotonic = now
        elapsed_s = max(now - self._start_monotonic, 0.0)

        row = [
            f"{elapsed_s:.3f}",
            f"{float(data['lateral_error_mm']):.6f}",
            *[f"{float(value):.3f}" for value in tcp_mm],
            *[f"{float(value):.3f}" for value in joints_deg],
        ]
        self._writer.writerow(row)
        self._file.flush()
        return True

    def stop(self):
        """Flush and close the active file, if any."""
        closed_path = None
        if self.active:
            closed_path = self.output_path
            self._file.flush()
            self._file.close()
        self._file = None
        self._writer = None
        self._start_monotonic = None
        return closed_path


class RCMDiagnosticsLogger(Node):
    """Save capture-scoped basic data."""

    def __init__(self):
        super().__init__("rcm_diagnostics_logger")
        self.declare_parameter("output_dir", "rcm_logs")
        self.declare_parameter("log_mode", "basic")
        self.output_dir = Path(
            self.get_parameter("output_dir").value
        ).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)

        requested_mode = str(self.get_parameter("log_mode").value).strip().lower()
        if requested_mode != "basic":
            raise ValueError(
                f"Unsupported RCM log_mode={requested_mode!r}; "
                "expected 'basic'"
            )
        self._basic_writer = BasicRCMSessionWriter(self.output_dir)
        self._event_subscription = self.create_subscription(
            String, "/neuro_final/rcm_log_event", self._on_log_event, 10,
        )
        self.get_logger().info("Basic RCM logging ready; waiting for RCM capture")

        self._subscription = self.create_subscription(
            String,
            "/neuro_final/rcm_diagnostics",
            self._on_diagnostics,
            10,
        )

    def _on_log_event(self, message):
        """Open/close basic files from the RCM state-machine lifecycle."""
        try:
            data = json.loads(message.data)
            event = data.get("event")
            if event == "start":
                csv_path, metadata_path = self._basic_writer.start(data)
                self.get_logger().info(f"Recording basic RCM data: {csv_path}")
                self.get_logger().info(f"RCM metadata: {metadata_path}")
            elif event == "stop":
                closed_path = self._basic_writer.stop()
                if closed_path is not None:
                    self.get_logger().info(f"Closed basic RCM data: {closed_path}")
        except (json.JSONDecodeError, TypeError, ValueError, OSError) as exc:
            self.get_logger().warning(f"Invalid RCM log event: {exc}")

    def _on_diagnostics(self, message):
        """Parse one diagnostics message for the selected output mode."""
        try:
            data = json.loads(message.data)
            self._basic_writer.write_sample(data)
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(
                f"Invalid RCM diagnostics message: {exc}"
            )

    def destroy_node(self):
        """Close the active capture file."""
        self._basic_writer.stop()
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
