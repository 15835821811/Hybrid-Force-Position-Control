"""Compare pre/post smoothing traces around the former six-second event."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np

from ..model import default_model_spec


def _task_kinematics(trace: dict[str, np.ndarray]) -> tuple[np.ndarray, ...]:
    task_time = np.asarray(trace["task_time"], dtype=np.float64)
    period = float(np.median(np.diff(task_time)))
    if "task_joint_velocity_rad_s" in trace:
        velocity = np.asarray(trace["task_joint_velocity_rad_s"], dtype=np.float64)
        acceleration = np.asarray(
            trace["task_joint_acceleration_rad_s2"], dtype=np.float64
        )
    else:
        stride = int(round(len(trace["time"]) / len(task_time)))
        velocity = np.asarray(trace["reference_dq"], dtype=np.float64)[
            stride - 1 :: stride
        ][: len(task_time)]
        acceleration = np.diff(
            np.vstack((np.zeros((1, velocity.shape[1])), velocity)), axis=0
        ) / period
    jerk = np.diff(acceleration, axis=0) / period
    return task_time, velocity, acceleration, task_time[1:], jerk


def _actual_kinematics(trace: dict[str, np.ndarray]) -> tuple[np.ndarray, ...]:
    spec = default_model_spec()
    model = spec.compile_model()
    _qpos_ids, dof_ids = spec.joint_addresses(model)
    time = np.asarray(trace["time"], dtype=np.float64)
    velocity = np.asarray(trace["qvel"], dtype=np.float64)[:, dof_ids]
    acceleration = np.gradient(velocity, time, axis=0, edge_order=2)
    jerk = np.gradient(acceleration, time, axis=0, edge_order=2)
    return time, velocity, acceleration, jerk


def _load(path: Path) -> dict[str, np.ndarray]:
    with np.load(path, allow_pickle=False) as archive:
        return {key: archive[key] for key in archive.files}


def compare_traces(
    before_path: Path,
    after_path: Path,
    output_dir: Path,
    *,
    window: tuple[float, float] = (5.5, 7.2),
) -> dict[str, Any]:
    before = _load(Path(before_path))
    after = _load(Path(after_path))
    data: dict[str, dict[str, np.ndarray]] = {}
    for label, trace in (("before", before), ("after", after)):
        task_time, task_velocity, task_acceleration, jerk_time, task_jerk = (
            _task_kinematics(trace)
        )
        actual_time, actual_velocity, actual_acceleration, actual_jerk = (
            _actual_kinematics(trace)
        )
        data[label] = {
            "task_time": task_time,
            "task_velocity": task_velocity,
            "task_acceleration": task_acceleration,
            "jerk_time": jerk_time,
            "task_jerk": task_jerk,
            "actual_time": actual_time,
            "actual_velocity": actual_velocity,
            "actual_acceleration": actual_acceleration,
            "actual_jerk": actual_jerk,
        }

    start, stop = map(float, window)
    before_jerk_mask = (
        (data["before"]["jerk_time"] >= start)
        & (data["before"]["jerk_time"] <= stop)
    )
    per_joint_peak = np.max(
        np.abs(data["before"]["task_jerk"][before_jerk_mask]), axis=0
    )
    selected_joints = np.argsort(per_joint_peak)[-3:][::-1]

    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 10,
            "axes.grid": True,
            "grid.alpha": 0.24,
            "axes.spines.top": False,
            "axes.spines.right": False,
        }
    )
    colors = ("#0072B2", "#D55E00", "#009E73")
    figure, axes = plt.subplots(3, 1, figsize=(10.5, 8.0), sharex=True)

    for color, joint in zip(colors, selected_joints):
        for label, style in (("before", "--"), ("after", "-")):
            values = data[label]
            mask = (
                (values["task_time"] >= start)
                & (values["task_time"] <= stop)
            )
            axes[0].plot(
                values["task_time"][mask],
                values["task_acceleration"][mask, joint],
                linestyle=style,
                color=color,
                linewidth=1.5,
                label=f"J{joint + 1} {label}",
            )
    axes[0].set_ylabel("Command accel. (rad/s²)")
    axes[0].legend(ncol=3, loc="upper left", fontsize=8)

    for label, style, color in (
        ("before", "--", "#D55E00"),
        ("after", "-", "#0072B2"),
    ):
        values = data[label]
        mask = (
            (values["jerk_time"] >= start)
            & (values["jerk_time"] <= stop)
        )
        envelope = np.max(np.abs(values["task_jerk"]), axis=1)
        axes[1].plot(
            values["jerk_time"][mask],
            envelope[mask],
            linestyle=style,
            color=color,
            linewidth=1.7,
            label=label,
        )
    axes[1].axhline(80.0, color="#009E73", linewidth=1.1, label="jerk limit")
    axes[1].set_ylabel("Max |command jerk| (rad/s³)")
    axes[1].legend(loc="upper left")

    for label, style, color in (
        ("before", "--", "#D55E00"),
        ("after", "-", "#0072B2"),
    ):
        values = data[label]
        mask = (
            (values["actual_time"] >= start)
            & (values["actual_time"] <= stop)
        )
        envelope = np.max(np.abs(values["actual_acceleration"]), axis=1)
        axes[2].plot(
            values["actual_time"][mask],
            envelope[mask],
            linestyle=style,
            color=color,
            linewidth=1.4,
            label=label,
        )
    axes[2].set_ylabel("Max |actual accel.| (rad/s²)")
    axes[2].set_xlabel("Time (s)")
    axes[2].legend(loc="upper left")
    axes[2].set_xlim(start, stop)
    figure.suptitle("Six-second joint-event comparison: hard cap vs smooth + jerk-limited")
    figure.tight_layout()

    output = Path(output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)
    png_path = output / "six_second_smoothing_comparison.png"
    pdf_path = output / "six_second_smoothing_comparison.pdf"
    figure.savefig(png_path, dpi=220, bbox_inches="tight")
    figure.savefig(pdf_path, bbox_inches="tight")
    plt.close(figure)

    metrics: dict[str, Any] = {
        "window_s": [start, stop],
        "selected_joints": [int(index + 1) for index in selected_joints],
        "before_trace": str(Path(before_path).resolve()),
        "after_trace": str(Path(after_path).resolve()),
        "figure_png": str(png_path),
        "figure_pdf": str(pdf_path),
    }
    for label, values in data.items():
        task_mask = (
            (values["task_time"] >= start)
            & (values["task_time"] <= stop)
        )
        jerk_mask = (
            (values["jerk_time"] >= start)
            & (values["jerk_time"] <= stop)
        )
        actual_mask = (
            (values["actual_time"] >= start)
            & (values["actual_time"] <= stop)
        )
        metrics[label] = {
            "maximum_command_acceleration_rad_s2": float(
                np.max(np.abs(values["task_acceleration"][task_mask]))
            ),
            "maximum_command_jerk_rad_s3": float(
                np.max(np.abs(values["task_jerk"][jerk_mask]))
            ),
            "maximum_actual_acceleration_rad_s2": float(
                np.max(np.abs(values["actual_acceleration"][actual_mask]))
            ),
            "maximum_actual_jerk_rad_s3": float(
                np.max(np.abs(values["actual_jerk"][actual_mask]))
            ),
        }
    metrics["reduction_percent"] = {
        key: 100.0 * (metrics["before"][key] - metrics["after"][key]) / metrics["before"][key]
        for key in metrics["before"]
    }
    report_path = output / "six_second_smoothing_comparison.json"
    report_path.write_text(
        json.dumps(metrics, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    metrics["report_json"] = str(report_path)
    return metrics


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before", type=Path, required=True)
    parser.add_argument("--after", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--window-start", type=float, default=5.5)
    parser.add_argument("--window-stop", type=float, default=7.2)
    args = parser.parse_args()
    report = compare_traces(
        args.before,
        args.after,
        args.output_dir,
        window=(args.window_start, args.window_stop),
    )
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
