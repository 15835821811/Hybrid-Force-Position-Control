"""Generate verified PSO and dynamic-tracking figures for one FPMFC run."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from paper_plot_style import COLORS, plt  # noqa: E402


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _save_bundle(figure: plt.Figure, output_stem: Path) -> dict[str, Any]:
    stem = Path(output_stem).resolve()
    stem.parent.mkdir(parents=True, exist_ok=True)
    outputs: dict[str, Any] = {}
    for suffix, options in (
        (".pdf", {}),
        (".png", {"dpi": 300}),
    ):
        path = stem.with_suffix(suffix)
        figure.savefig(path, **options)
        if not path.is_file() or path.stat().st_size == 0:
            raise RuntimeError(f"figure was not written: {path}")
        outputs[suffix[1:]] = {
            "path": str(path),
            "bytes": path.stat().st_size,
            "sha256": _sha256(path),
        }
    plt.close(figure)
    return outputs


def _optimizer_path(summary_path: Path, variant: str, run: dict[str, Any]) -> Path:
    recorded = Path(run["path"])
    if recorded.is_file():
        return recorded.resolve()
    fallback = summary_path.parent / variant / f"pso_seed_{int(run['seed']):02d}.json"
    if not fallback.is_file():
        raise FileNotFoundError(fallback)
    return fallback.resolve()


def _panel_label(axis: Any, label: str) -> None:
    axis.text(
        -0.14,
        1.04,
        label,
        transform=axis.transAxes,
        fontweight="bold",
        va="bottom",
        ha="left",
    )


def generate_optimization_figure(
    suite_summary: Path, output_stem: Path
) -> tuple[dict[str, Any], dict[str, Any]]:
    summary_path = Path(suite_summary).resolve()
    payload = json.loads(summary_path.read_text(encoding="utf-8"))
    if "joint" not in payload["variants"]:
        raise ValueError("suite summary has no joint variant")
    variant = payload["variants"]["joint"]
    runs = variant["runs"]
    if not runs:
        raise ValueError("joint suite has no runs")

    histories: list[np.ndarray] = []
    optimizer_paths: list[Path] = []
    seed_values: list[int] = []
    objectives: list[float] = []
    capture_times: list[float] = []
    arm_angles_deg: list[float] = []
    feasible: list[bool] = []
    selected_seed = int(variant["selected_seed"])
    for run in runs:
        path = _optimizer_path(summary_path, "joint", run)
        optimizer_paths.append(path)
        result = json.loads(path.read_text(encoding="utf-8"))["optimizer"]
        history = np.asarray(result["history"], dtype=np.float64)
        if history.ndim != 1 or not len(history) or not np.all(np.isfinite(history)):
            raise ValueError(f"invalid optimizer history: {path}")
        histories.append(history)
        seed_values.append(int(run["seed"]))
        objectives.append(float(run["best_objective"]))
        capture_times.append(float(run["capture_time_s"]))
        arm_angles_deg.append(float(np.rad2deg(run["terminal_arm_angle_rad"])))
        feasible.append(bool(run["feasible"]))

    max_generations = max(len(values) for values in histories)
    padded = np.vstack(
        [np.pad(values, (0, max_generations - len(values)), mode="edge") for values in histories]
    )
    generations = np.arange(1, max_generations + 1)
    median = np.median(padded, axis=0)
    lower = np.quantile(padded, 0.25, axis=0)
    upper = np.quantile(padded, 0.75, axis=0)

    figure, axes = plt.subplots(1, 3, figsize=(7.15, 2.35))
    for seed, history in zip(seed_values, histories):
        selected = seed == selected_seed
        axes[0].step(
            np.arange(1, len(history) + 1),
            history,
            where="post",
            color=COLORS[1] if selected else COLORS[0],
            alpha=0.95 if selected else 0.22,
            linewidth=1.5 if selected else 0.75,
            zorder=3 if selected else 1,
        )
    axes[0].fill_between(generations, lower, upper, color=COLORS[2], alpha=0.18)
    axes[0].step(generations, median, where="post", color=COLORS[2], linewidth=1.5)
    positive = padded[padded > 0.0]
    if positive.size and float(np.max(positive) / np.min(positive)) >= 100.0:
        axes[0].set_yscale("log")
    axes[0].set_xlabel("PSO generation")
    axes[0].set_ylabel("Best-so-far objective")
    _panel_label(axes[0], "(a)")

    seeds = np.asarray(seed_values)
    final_objectives = np.asarray(objectives)
    feasible_array = np.asarray(feasible)
    axes[1].scatter(
        seeds[feasible_array],
        final_objectives[feasible_array],
        marker="o",
        s=23,
        color=COLORS[0],
        label="Feasible",
    )
    if np.any(~feasible_array):
        axes[1].scatter(
            seeds[~feasible_array],
            final_objectives[~feasible_array],
            marker="x",
            s=28,
            color=COLORS[1],
            label="Infeasible",
        )
    selected_index = seed_values.index(selected_seed)
    axes[1].scatter(
        [selected_seed],
        [final_objectives[selected_index]],
        marker="*",
        s=78,
        color=COLORS[1],
        edgecolors="none",
        label="Selected",
        zorder=4,
    )
    axes[1].axhline(np.median(final_objectives), color=COLORS[2], linestyle="--", linewidth=1.0)
    axes[1].set_xlabel("Random seed")
    axes[1].set_ylabel("Final best objective")
    axes[1].set_xticks(seeds)
    axes[1].legend(frameon=False, loc="best")
    _panel_label(axes[1], "(b)")

    points = axes[2].scatter(
        capture_times,
        arm_angles_deg,
        c=final_objectives,
        cmap="viridis",
        s=28,
        marker="o",
    )
    axes[2].scatter(
        [capture_times[selected_index]],
        [arm_angles_deg[selected_index]],
        marker="*",
        s=90,
        color=COLORS[1],
        edgecolors="none",
        zorder=4,
    )
    axes[2].set_xlabel("Capture time $T$ (s)")
    axes[2].set_ylabel(r"Terminal arm angle $\psi$ (deg)")
    colorbar = figure.colorbar(points, ax=axes[2], pad=0.02, fraction=0.08)
    colorbar.set_label("Objective")
    _panel_label(axes[2], "(c)")
    figure.tight_layout(w_pad=1.0, pad=0.25)

    outputs = _save_bundle(figure, output_stem)
    summary = {
        "seed_count": len(runs),
        "feasible_seed_count": int(np.count_nonzero(feasible_array)),
        "selected_seed": selected_seed,
        "best_objective": float(np.min(final_objectives)),
        "objective_mean": float(np.mean(final_objectives)),
        "objective_std": float(np.std(final_objectives)),
        "capture_time_s": float(capture_times[selected_index]),
        "terminal_arm_angle_deg": float(arm_angles_deg[selected_index]),
        "optimizer_files": [
            {"path": str(path), "sha256": _sha256(path)} for path in optimizer_paths
        ],
    }
    return outputs, summary


def _base_orientation_drift_rad(base_quaternion: np.ndarray) -> np.ndarray:
    quaternion = np.asarray(base_quaternion, dtype=np.float64)
    quaternion = quaternion / np.linalg.norm(quaternion, axis=1, keepdims=True)
    reference = quaternion[0]
    cosine_half = np.clip(np.abs(quaternion @ reference), 0.0, 1.0)
    return 2.0 * np.arccos(cosine_half)


def generate_tracking_figure(
    trace_path: Path,
    metrics_path: Path,
    output_stem: Path,
) -> tuple[dict[str, Any], dict[str, Any]]:
    trace_file = Path(trace_path).resolve()
    metrics_file = Path(metrics_path).resolve()
    with np.load(trace_file, allow_pickle=False) as archive:
        trace = {key: archive[key] for key in archive.files}
    metrics = json.loads(metrics_file.read_text(encoding="utf-8"))
    time_s = np.asarray(trace["time"], dtype=np.float64)
    actual_position = np.asarray(trace["flange_position"], dtype=np.float64)
    desired_position = np.asarray(trace["desired_position"], dtype=np.float64)
    position_error_mm = 1000.0 * np.asarray(trace["position_error_m"], dtype=np.float64)
    orientation_error_deg = np.rad2deg(
        np.asarray(trace["orientation_error_rad"], dtype=np.float64)
    )
    arm_angle_deg = np.rad2deg(np.unwrap(np.asarray(trace["arm_angle_rad"], dtype=np.float64)))
    desired_arm_angle_deg = np.rad2deg(
        np.unwrap(np.asarray(trace["desired_arm_angle_rad"], dtype=np.float64))
    )
    base_position = np.asarray(trace["base_qpos"], dtype=np.float64)[:, :3]
    base_translation_mm = 1000.0 * np.linalg.norm(base_position - base_position[0], axis=1)
    base_orientation_deg = np.rad2deg(
        _base_orientation_drift_rad(np.asarray(trace["base_qpos"], dtype=np.float64)[:, 3:7])
    )
    base_angular_speed = np.linalg.norm(
        np.asarray(trace["base_twist"], dtype=np.float64)[:, 3:], axis=1
    )
    clearance_mm = 1000.0 * np.asarray(trace["minimum_clearance_m"], dtype=np.float64)
    acceptance = metrics["effective_config"]["acceptance"]
    position_limit_mm = 1000.0 * float(acceptance["terminal_position_error_m"])
    orientation_limit_deg = float(
        np.rad2deg(acceptance["terminal_orientation_error_rad"])
    )
    clearance_limit_mm = 1000.0 * float(metrics["clearance_thresholds_m"]["acceptance"])

    figure, axes = plt.subplots(2, 4, figsize=(7.15, 4.35))
    coordinate_labels = ("x", "y", "z")
    coordinate_handles = []
    for index, label in enumerate(coordinate_labels):
        color = COLORS[index]
        handle = axes[0, 0].plot(
            time_s, actual_position[:, index], color=color, label=label
        )[0]
        coordinate_handles.append(handle)
        axes[0, 0].plot(
            time_s,
            desired_position[:, index],
            color=color,
            linestyle="--",
            linewidth=1.0,
        )
    axes[0, 0].set_xlabel("Time (s)")
    axes[0, 0].set_ylabel("Flange position (m)")
    axes[0, 0].legend(
        coordinate_handles,
        coordinate_labels,
        frameon=False,
        ncol=3,
        columnspacing=0.7,
        handlelength=1.5,
        loc="upper left",
    )
    _panel_label(axes[0, 0], "(a)")

    axes[0, 1].plot(time_s, position_error_mm, color=COLORS[0])
    axes[0, 1].axhline(
        position_limit_mm,
        color=COLORS[1],
        linestyle="--",
        linewidth=1.0,
        label="Terminal limit",
    )
    axes[0, 1].set_xlabel("Time (s)")
    axes[0, 1].set_ylabel("Position error (mm)")
    axes[0, 1].legend(frameon=False, loc="upper left")
    _panel_label(axes[0, 1], "(b)")

    axes[0, 2].plot(time_s, orientation_error_deg, color=COLORS[1])
    axes[0, 2].axhline(
        orientation_limit_deg,
        color=COLORS[1],
        linestyle="--",
        linewidth=1.0,
        label="Terminal limit",
    )
    axes[0, 2].set_xlabel("Time (s)")
    axes[0, 2].set_ylabel("Orientation error (deg)")
    axes[0, 2].legend(frameon=False, loc="upper left")
    _panel_label(axes[0, 2], "(c)")

    axes[0, 3].plot(time_s, arm_angle_deg, color=COLORS[2], label="Actual")
    axes[0, 3].plot(
        time_s,
        desired_arm_angle_deg,
        color=COLORS[1],
        linestyle="--",
        linewidth=1.0,
        label="Desired",
    )
    axes[0, 3].set_xlabel("Time (s)")
    axes[0, 3].set_ylabel(r"Arm angle $\psi$ (deg)")
    axes[0, 3].legend(frameon=False)
    _panel_label(axes[0, 3], "(d)")

    axes[1, 0].plot(time_s, base_translation_mm, color=COLORS[3])
    axes[1, 0].set_xlabel("Time (s)")
    axes[1, 0].set_ylabel("Base translation drift (mm)")
    _panel_label(axes[1, 0], "(e)")

    axes[1, 1].plot(time_s, base_orientation_deg, color=COLORS[4])
    axes[1, 1].set_xlabel("Time (s)")
    axes[1, 1].set_ylabel("Base attitude drift (deg)")
    _panel_label(axes[1, 1], "(f)")

    axes[1, 2].plot(time_s, base_angular_speed, color=COLORS[5])
    axes[1, 2].set_xlabel("Time (s)")
    axes[1, 2].set_ylabel(r"Base angular speed (rad/s)")
    _panel_label(axes[1, 2], "(g)")

    axes[1, 3].plot(time_s, clearance_mm, color=COLORS[2])
    axes[1, 3].axhline(
        clearance_limit_mm,
        color=COLORS[1],
        linestyle="--",
        linewidth=1.0,
        label="Acceptance limit",
    )
    axes[1, 3].set_xlabel("Time (s)")
    axes[1, 3].set_ylabel("Minimum clearance (mm)")
    axes[1, 3].legend(frameon=False, loc="lower left")
    _panel_label(axes[1, 3], "(h)")
    figure.tight_layout(w_pad=0.9, h_pad=1.1, pad=0.35)

    outputs = _save_bundle(figure, output_stem)
    target_outward = np.asarray(trace["target_grasp_position"][-1], dtype=np.float64) - np.asarray(
        trace["target_center_position"][-1], dtype=np.float64
    )
    target_outward /= np.linalg.norm(target_outward)
    flange_outward = np.asarray(trace["flange_rotation"][-1], dtype=np.float64)[:, 2]
    face_normal_dot = float(target_outward @ flange_outward)
    source_trace_sha256 = _sha256(trace_file)
    summary = {
        "trace_path": str(trace_file),
        "trace_sha256": source_trace_sha256,
        "metrics_path": str(metrics_file),
        "metrics_sha256": _sha256(metrics_file),
        "metrics_trace_hash_matches": metrics["trace"]["sha256"] == source_trace_sha256,
        "acceptance_passed": bool(metrics["acceptance"]["passed"]),
        "terminal_face_normal_dot": face_normal_dot,
        "terminal_face_to_face_error_deg": float(
            np.rad2deg(np.arccos(np.clip(-face_normal_dot, -1.0, 1.0)))
        ),
        "terminal_position_error_mm": float(position_error_mm[-1]),
        "terminal_orientation_error_deg": float(orientation_error_deg[-1]),
        "maximum_base_translation_drift_mm": float(np.max(base_translation_mm)),
        "maximum_base_attitude_drift_deg": float(np.max(base_orientation_deg)),
        "maximum_base_angular_speed_rad_s": float(np.max(base_angular_speed)),
        "minimum_clearance_mm": float(np.min(clearance_mm)),
    }
    return outputs, summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite-summary", type=Path, required=True)
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--metrics", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    output_dir = args.output_dir.resolve()
    optimization_outputs, optimization_summary = generate_optimization_figure(
        args.suite_summary,
        output_dir / "fig_optimization_process",
    )
    tracking_outputs, tracking_summary = generate_tracking_figure(
        args.trace,
        args.metrics,
        output_dir / "fig_tracking_error_base_drift",
    )
    report = {
        "material_passport": {
            "origin_skill": "paper-figure",
            "origin_mode": "experiment-visualization",
            "verification_status": "VERIFIED",
            "version_label": "fpmfc_full_process_figures_v1",
        },
        "suite_summary": {
            "path": str(args.suite_summary.resolve()),
            "sha256": _sha256(args.suite_summary.resolve()),
        },
        "optimization": optimization_summary,
        "tracking": tracking_summary,
        "outputs": {
            "optimization_process": optimization_outputs,
            "tracking_error_base_drift": tracking_outputs,
        },
    }
    checks = {
        "has_feasible_seed": optimization_summary["feasible_seed_count"] > 0,
        "dynamic_acceptance_passed": tracking_summary["acceptance_passed"],
        "trace_hash_matches_metrics": tracking_summary["metrics_trace_hash_matches"],
        "terminal_faces_opposed": tracking_summary["terminal_face_normal_dot"] <= -0.99999,
        "all_outputs_nonempty": all(
            output["bytes"] > 0
            for bundle in report["outputs"].values()
            for output in bundle.values()
        ),
    }
    report["checks"] = checks
    report["passed"] = all(checks.values())
    output_dir.mkdir(parents=True, exist_ok=True)
    manifest = output_dir / "full_process_visualization_manifest.json"
    manifest.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    if not report["passed"]:
        raise RuntimeError(f"full-process visualization validation failed: {checks}")
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
