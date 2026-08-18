"""Analyze repeated Studio Sim RCM experiments."""
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt


DATA_DIR = Path(
    "rcm_logs/studio_calibrated_standard/"
    "full_stick_0p5_deg_s"
)

OUTPUT_DIR = DATA_DIR / "analysis"
RUN_COUNT = 5
STEADY_THRESHOLD_DEG_S = 0.36


def find_csv_files():
    """Find exactly one CSV file for each formal run."""

    files = []

    for run_number in range(1, RUN_COUNT + 1):
        run_dir = DATA_DIR / f"run_{run_number:02d}"
        matches = sorted(run_dir.glob("*.csv"))

        if len(matches) != 1:
            raise RuntimeError(
                f"{run_dir}: expected one CSV, found {len(matches)}"
            )

        files.append(matches[0])

    return files


def vector_norm(row, prefix):
    """Return XYZ vector magnitude."""

    return math.sqrt(
        sum(float(row[f"{prefix}{axis}_rad_s"]) ** 2 for axis in "xyz")
    )


def load_steady_rows(path):
    """Load samples from the stable rotation period."""

    with path.open(newline="") as file:
        rows = list(csv.DictReader(file))

    return [
        row
        for row in rows
        if math.degrees(vector_norm(row, "desired_w"))
        > STEADY_THRESHOLD_DEG_S
    ]


def summarize_run(run_number, rows):
    """Calculate metrics for one stable rotation period."""

    errors = [float(row["lateral_error_mm"]) for row in rows]
    commands = [
        math.degrees(vector_norm(row, "desired_w"))
        for row in rows
    ]

    qdot = [
        abs(float(row[f"qdot{i}_rad_s"]))
        for row in rows
        for i in range(1, 8)
    ]

    angular_errors = []
    for row in rows:
        difference = [
            float(row[f"desired_w{axis}_rad_s"])
            - float(row[f"achieved_w{axis}_rad_s"])
            for axis in "xyz"
        ]
        angular_errors.append(
            math.sqrt(sum(value**2 for value in difference))
        )

    duration = float(rows[-1]["time_s"]) - float(rows[0]["time_s"])

    return {
        "run": f"run_{run_number:02d}",
        "samples": len(rows),
        "duration_s": duration,
        "mean_command_deg_s": sum(commands) / len(commands),
        "mean_error_mm": sum(errors) / len(errors),
        "rms_error_mm": math.sqrt(
            sum(value**2 for value in errors) / len(errors)
        ),
        "max_error_mm": max(errors),
        "max_joint_speed_rad_s": max(qdot),
        "mean_angular_error_rad_s": (
            sum(angular_errors) / len(angular_errors)
        ),
        "limited_samples": sum(
            int(row["limited"]) for row in rows
        ),
    }


def save_summary(summaries):
    """Save one summary row per formal run."""

    path = OUTPUT_DIR / "run_summary.csv"

    with path.open("w", newline="") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=summaries[0].keys(),
        )
        writer.writeheader()
        writer.writerows(summaries)

    return path


def plot_error_time(datasets):
    """Plot stable RCM error over time."""

    figure, axis = plt.subplots(
        figsize=(8, 4.5),
        constrained_layout=True,
    )

    for run_name, rows in datasets:
        start_time = float(rows[0]["time_s"])

        time_s = [
            float(row["time_s"]) - start_time
            for row in rows
        ]
        error_mm = [
            float(row["lateral_error_mm"])
            for row in rows
        ]

        axis.plot(
            time_s,
            error_mm,
            label=run_name,
            linewidth=1.2,
        )

    axis.set(
        title="Model-Based RCM Error During Studio Sim Rotation",
        xlabel="Time (s)",
        ylabel="Model-based lateral RCM error (mm)",
    )
    axis.grid(True, alpha=0.3)
    axis.legend(ncol=3)

    path = OUTPUT_DIR / "rcm_error_time.png"
    figure.savefig(path, dpi=200)
    plt.close(figure)

    return path


def plot_error_boxplot(datasets):
    """Compare model-based RCM error distributions."""

    labels = []
    errors_mm = []

    for run_name, rows in datasets:
        labels.append(run_name)
        errors_mm.append([
            float(row["lateral_error_mm"])
            for row in rows
        ])

    figure, axis = plt.subplots(
        figsize=(7, 4.5),
        constrained_layout=True,
    )

    boxes = axis.boxplot(
        errors_mm,
        labels=labels,
        patch_artist=True,
        showfliers=True,
    )

    for box in boxes["boxes"]:
        box.set(facecolor="#8ecae6", alpha=0.8)

    axis.set(
        title="Repeated Studio Sim RCM Experiments",
        xlabel="Experiment",
        ylabel="Model-based lateral RCM error (mm)",
    )
    axis.grid(True, axis="y", alpha=0.3)

    path = OUTPUT_DIR / "rcm_error_boxplot.png"
    figure.savefig(path, dpi=200)
    plt.close(figure)

    return path


def save_steady_samples(datasets):
    """Combine stable samples from all runs."""

    path = OUTPUT_DIR / "steady_samples.csv"
    original_fields = list(datasets[0][1][0].keys())

    fieldnames = [
        "run",
        "elapsed_s",
        *original_fields,
    ]

    with path.open("w", newline="") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=fieldnames,
        )
        writer.writeheader()

        for run_name, rows in datasets:
            start_time = float(rows[0]["time_s"])

            for row in rows:
                output = {
                    "run": run_name,
                    "elapsed_s": (
                        float(row["time_s"]) - start_time
                    ),
                    **row,
                }
                writer.writerow(output)

    return path


def main():
    files = find_csv_files()
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    summaries = []
    datasets = []

    for run_number, path in enumerate(files, start=1):
        rows = load_steady_rows(path)

        if not rows:
            raise RuntimeError(
                f"No steady samples found in {path}"
            )

        summary = summarize_run(run_number, rows)
        summaries.append(summary)
        datasets.append((summary["run"], rows))

        print(
            f"{summary['run']}: "
            f"RMS={summary['rms_error_mm']:.6f} mm, "
            f"Max={summary['max_error_mm']:.6f} mm"
        )

    summary_path = save_summary(summaries)
    samples_path = save_steady_samples(datasets)
    time_plot_path = plot_error_time(datasets)
    boxplot_path = plot_error_boxplot(datasets)

    print(f"Saved: {summary_path}")
    print(f"Saved: {samples_path}")
    print(f"Saved: {time_plot_path}")
    print(f"Saved: {boxplot_path}")


if __name__ == "__main__":
    main()
