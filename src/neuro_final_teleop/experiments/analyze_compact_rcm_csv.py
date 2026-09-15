"""Calculate summary metrics from one compact RCM CSV file."""

import argparse
import csv
from pathlib import Path

import numpy as np


FIELDS = (
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


def load_rows(path):
    """Load and validate one compact RCM CSV file."""

    with path.open(newline="") as file:
        reader = csv.DictReader(file)
        missing = set(FIELDS) - set(reader.fieldnames or ())

        if missing:
            names = ", ".join(sorted(missing))
            raise ValueError(f"Missing compact CSV fields: {names}")

        rows = list(reader)

    if not rows:
        raise ValueError("CSV contains no data rows")

    return rows


def values(rows, field):
    """Convert one CSV column to a NumPy float array."""

    return np.asarray([float(row[field]) for row in rows])


def summarize(rows):
    """Calculate the core metrics used by the RCM report."""

    time_s = values(rows, "time_s")
    error = values(rows, "lateral_error_mm")
    angular_error = values(rows, "angular_error_rad_s")
    target_insertion = values(rows, "target_insertion_mm_s")
    achieved_insertion = values(rows, "achieved_insertion_mm_s")
    depth = values(rows, "insertion_depth_mm")
    joint_speed = values(rows, "max_joint_speed_rad_s")

    duration_s = max(float(time_s[-1] - time_s[0]), 0.0)
    sample_rate_hz = (
        (len(rows) - 1) / duration_s
        if duration_s > 0.0 and len(rows) > 1
        else 0.0
    )

    return {
        "samples": len(rows),
        "duration_s": duration_s,
        "sample_rate_hz": sample_rate_hz,
        "mean_rcm_error_mm": float(np.mean(error)),
        "rms_rcm_error_mm": float(np.sqrt(np.mean(error**2))),
        "p95_rcm_error_mm": float(np.percentile(error, 95)),
        "max_rcm_error_mm": float(np.max(error)),
        "mean_angular_error_rad_s": float(np.mean(angular_error)),
        "mean_insertion_error_mm_s": float(
            np.mean(np.abs(target_insertion - achieved_insertion))
        ),
        "minimum_depth_mm": float(np.min(depth)),
        "maximum_depth_mm": float(np.max(depth)),
        "maximum_joint_speed_rad_s": float(np.max(joint_speed)),
        "travel_limited_samples": int(np.sum(values(rows, "travel_limited"))),
        "velocity_limited_samples": int(
            np.sum(values(rows, "joint_velocity_limited"))
        ),
        "acceleration_limited_samples": int(
            np.sum(values(rows, "joint_acceleration_limited"))
        ),
    }


def print_summary(path, metrics):
    """Print one compact, report-ready summary."""

    print(f"Compact RCM summary: {path}")
    print(f"  Samples:                    {metrics['samples']}")
    print(f"  Duration:                   {metrics['duration_s']:.3f} s")
    print(f"  Sample rate:                {metrics['sample_rate_hz']:.2f} Hz")
    print(f"  Mean RCM error:             {metrics['mean_rcm_error_mm']:.6f} mm")
    print(f"  RMS RCM error:              {metrics['rms_rcm_error_mm']:.6f} mm")
    print(f"  95th-percentile RCM error:  {metrics['p95_rcm_error_mm']:.6f} mm")
    print(f"  Maximum RCM error:          {metrics['max_rcm_error_mm']:.6f} mm")
    print(f"  Mean angular error:         {metrics['mean_angular_error_rad_s']:.6f} rad/s")
    print(f"  Mean insertion error:       {metrics['mean_insertion_error_mm_s']:.6f} mm/s")
    print(f"  Insertion depth range:      {metrics['minimum_depth_mm']:.3f} to "
          f"{metrics['maximum_depth_mm']:.3f} mm")
    print(f"  Maximum joint speed:        {metrics['maximum_joint_speed_rad_s']:.6f} rad/s")
    print(f"  Travel limited samples:     {metrics['travel_limited_samples']}")
    print(f"  Velocity limited samples:   {metrics['velocity_limited_samples']}")
    print(f"  Acceleration limited:       {metrics['acceleration_limited_samples']}")


def main():
    parser = argparse.ArgumentParser(
        description="Analyze one compact RCM diagnostics CSV file."
    )
    parser.add_argument("csv_file", type=Path)
    args = parser.parse_args()

    rows = load_rows(args.csv_file)
    print_summary(args.csv_file, summarize(rows))


if __name__ == "__main__":
    main()
