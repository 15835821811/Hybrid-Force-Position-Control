"""Generate publication-ready plots and five synchronized MuJoCo videos."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
from matplotlib import font_manager
from PIL import Image, ImageDraw, ImageFont

from .model import PROJECT_ROOT, default_model_spec, geom_id


COLORS = ("#0072B2", "#D55E00", "#009E73", "#CC79A7", "#E69F00")
TARGET_FRAME_AXIS_LENGTH_M = 0.160
END_EFFECTOR_FRAME_AXIS_LENGTH_M = 0.100
WAYPOINT_FRAME_AXIS_LENGTH_M = 0.050
VIDEO_WAYPOINT_FRAME_AXIS_LENGTH_M = 0.055
AXIS_COLORS_RGBA = (
    np.asarray([1.00, 0.12, 0.12, 1.0]),
    np.asarray([0.12, 1.00, 0.20, 1.0]),
    np.asarray([0.15, 0.42, 1.00, 1.0]),
)


@dataclass(frozen=True)
class CameraView:
    key: str
    title: str
    azimuth_deg: float
    elevation_deg: float
    distance_m: float


VIEWS = (
    CameraView("isometric", "Isometric", 135.0, -24.0, 2.05),
    CameraView("front", "Front", 180.0, -8.0, 2.05),
    CameraView("right", "Right side", 90.0, -10.0, 2.05),
    CameraView("top", "Top", 180.0, -89.0, 2.00),
    CameraView("rear", "Rear", 0.0, -14.0, 2.05),
)


@dataclass(frozen=True)
class VideoConfig:
    scenario_index: int = 0
    width: int = 640
    height: int = 480
    fps: int = 30
    crf: int = 20


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_npz(path: Path) -> dict[str, np.ndarray]:
    with np.load(path, allow_pickle=False) as archive:
        return {key: archive[key] for key in archive.files}


def _base_drift(trace: dict[str, np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    base = trace["base_qpos"]
    initial = trace["initial_qpos"][:7]
    translation = np.linalg.norm(base[:, :3] - initial[None, :3], axis=1)
    quaternions = base[:, 3:7]
    quaternions = quaternions / np.maximum(np.linalg.norm(quaternions, axis=1, keepdims=True), 1e-12)
    reference = initial[3:7] / max(float(np.linalg.norm(initial[3:7])), 1e-12)
    orientation = 2.0 * np.arccos(np.clip(np.abs(quaternions @ reference), -1.0, 1.0))
    return translation, orientation


def _style() -> None:
    plt.style.use("seaborn-v0_8-whitegrid")
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "axes.titlesize": 12,
            "axes.labelsize": 10,
            "legend.fontsize": 8,
            "figure.facecolor": "white",
            "savefig.facecolor": "white",
            "axes.spines.top": False,
            "axes.spines.right": False,
        }
    )


def _save_figure(fig: plt.Figure, path: Path) -> dict[str, Any]:
    fig.savefig(path, dpi=180, bbox_inches="tight")
    plt.close(fig)
    with Image.open(path) as image:
        width, height = image.size
    return {"path": str(path.relative_to(PROJECT_ROOT)), "sha256": _sha256(path), "width": width, "height": height, "bytes": path.stat().st_size}


def _tracking_paths(traces: list[dict[str, np.ndarray]], metrics: dict[str, Any], output: Path) -> dict[str, Any]:
    fig = plt.figure(figsize=(16, 8.7), constrained_layout=True)
    all_points = np.concatenate([np.vstack((trace["flange_position"], trace["target_position"])) for trace in traces], axis=0)
    center = 0.5 * (np.min(all_points, axis=0) + np.max(all_points, axis=0))
    radius = 0.65 * float(np.max(np.ptp(all_points, axis=0))) + 0.04
    for index, trace in enumerate(traces):
        axis = fig.add_subplot(2, 3, index + 1, projection="3d")
        actual = trace["flange_position"]
        target = trace["target_position"]
        waypoints = np.asarray(metrics["scenarios"][index]["target_contract"]["waypoint_points_m"])
        target_rotation = np.asarray(metrics["scenarios"][index]["target_contract"]["target_rotation_world"])
        axis.plot(*target.T, color="#172B4D", lw=2.3, ls="--", label="Reference")
        axis.plot(*actual.T, color=COLORS[index], lw=1.5, label="Flexiv flange")
        axis.scatter(*waypoints.T, color="#00A878", edgecolor="white", s=34, zorder=5, label="Waypoints")
        for waypoint_index, waypoint in enumerate(waypoints, start=1):
            axis.text(*waypoint, f" W{waypoint_index}", fontsize=7, color="#475569")
            _plot_coordinate_frame(axis, waypoint, target_rotation, WAYPOINT_FRAME_AXIS_LENGTH_M, 0.34, 0.8)
        _plot_coordinate_frame(axis, target[-1], target_rotation, 0.120, 0.55, 1.4)
        _plot_coordinate_frame(axis, actual[-1], trace["flange_rotation"][-1], 0.075, 1.0, 1.8)
        axis.scatter(*actual[0], color="#3B82F6", s=28)
        axis.scatter(*actual[-1], color="#EF4444", s=28)
        axis.set_xlim(center[0] - radius, center[0] + radius)
        axis.set_ylim(center[1] - radius, center[1] + radius)
        axis.set_zlim(center[2] - radius, center[2] + radius)
        axis.set_xlabel("X [m]")
        axis.set_ylabel("Y [m]")
        axis.set_zlabel("Z [m]")
        axis.set_title(f"Scenario {index:02d} | seed {metrics['scenarios'][index]['seed']}")
        axis.view_init(elev=25, azim=-58)
        axis.legend(loc="upper left")
    summary = fig.add_subplot(2, 3, 6)
    summary.axis("off")
    aggregate = metrics["aggregate"]
    summary.text(
        0.04,
        0.92,
        "Five-scenario tracking summary",
        fontsize=17,
        weight="bold",
        color="#172B4D",
        va="top",
    )
    summary.text(
        0.04,
        0.73,
        "\n".join(
            (
                f"QP success rate        {100.0 * aggregate['qp_success_rate_min']:.1f}%",
                f"Worst RMS error        {1000.0 * aggregate['tracking_rms_m_max']:.3f} mm",
                f"Worst peak error       {1000.0 * aggregate['tracking_max_m_max']:.3f} mm",
                f"Minimum clearance      {1000.0 * aggregate['minimum_signed_clearance_m']:.1f} mm",
                f"Worst QP p99 latency   {aggregate['qp_latency_p99_ms_max']:.3f} ms",
            )
        ),
        fontsize=13,
        family="monospace",
        linespacing=1.8,
        color="#334155",
        va="top",
    )
    summary.text(0.04, 0.20, "Start", color="#3B82F6", fontsize=11, weight="bold")
    summary.text(0.22, 0.20, "End", color="#EF4444", fontsize=11, weight="bold")
    fig.suptitle("Free-floating Flexiv Rizon 4s — tracking paths | RGB = XYZ; long/faint target, short/solid EE", fontsize=18, weight="bold")
    return _save_figure(fig, output / "tracking_paths_3d.png")


def _error_curves(traces: list[dict[str, np.ndarray]], output: Path) -> dict[str, Any]:
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True, constrained_layout=True)
    for index, trace in enumerate(traces):
        axes[0].plot(trace["time"], 1000.0 * trace["position_error"], color=COLORS[index], lw=1.2, label=f"Scenario {index:02d}")
        axes[1].plot(trace["time"], np.rad2deg(trace["orientation_error_rad"]), color=COLORS[index], lw=1.2)
    axes[0].set_ylabel("Position error [mm]")
    axes[0].set_title("Flange position tracking error")
    axes[0].legend(ncol=5, loc="upper right")
    axes[1].set_ylabel("Orientation error [deg]")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_title("Flange orientation tracking error")
    for axis in axes:
        axis.axvline(4.5, color="#64748B", ls="--", lw=1.0, alpha=0.8)
        axis.text(4.62, 0.92, "path start", transform=axis.get_xaxis_transform(), color="#64748B", fontsize=9)
    fig.suptitle("Tracking error across five deterministic scenarios", fontsize=18, weight="bold")
    return _save_figure(fig, output / "tracking_error_curves.png")


def _base_drift_figure(traces: list[dict[str, np.ndarray]], output: Path) -> dict[str, Any]:
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True, constrained_layout=True)
    for index, trace in enumerate(traces):
        translation, orientation = _base_drift(trace)
        axes[0].plot(trace["time"], 1000.0 * translation, color=COLORS[index], lw=1.25, label=f"Scenario {index:02d}")
        axes[1].plot(trace["time"], np.rad2deg(orientation), color=COLORS[index], lw=1.25)
    axes[0].set_ylabel("Translation drift [mm]")
    axes[0].set_title("Free-base translation drift from initial pose")
    axes[0].legend(ncol=5, loc="upper left")
    axes[1].set_ylabel("Quaternion geodesic drift [deg]")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_title("Free-base orientation drift from initial pose")
    fig.suptitle("Spacecraft-base reaction motion", fontsize=18, weight="bold")
    return _save_figure(fig, output / "base_pose_drift.png")


def _safety_qp_figure(traces: list[dict[str, np.ndarray]], output: Path) -> dict[str, Any]:
    fig, axes = plt.subplots(2, 2, figsize=(15, 9), constrained_layout=True)
    for index, trace in enumerate(traces):
        color = COLORS[index]
        stride = 10
        axes[0, 0].plot(trace["time"][::stride], 1000.0 * trace["minimum_clearance"][::stride], color=color, lw=1.0, label=f"S{index:02d}")
        residual = np.linalg.norm(trace["momentum"] - trace["momentum"][0], axis=1)
        axes[0, 1].plot(trace["time"][::stride], residual[::stride], color=color, lw=1.0)
        axes[1, 0].plot(trace["task_time"], 1000.0 * trace["task_full_latency"], color=color, lw=1.0)
        axes[1, 1].plot(trace["task_time"], trace["task_iterations"], color=color, lw=1.0)
    axes[0, 0].axhline(40.0, color="#DC2626", ls="--", lw=1.2, label="QP safe margin")
    axes[0, 0].set(title="Minimum whole-body signed clearance", ylabel="Clearance [mm]")
    axes[0, 0].legend(ncol=3)
    axes[0, 1].set(title="Total momentum numerical residual", ylabel="Residual norm")
    axes[1, 0].axhline(20.0, color="#DC2626", ls="--", lw=1.2, label="20 ms task budget")
    axes[1, 0].set(title="One-QP full latency", xlabel="Time [s]", ylabel="Latency [ms]")
    axes[1, 0].legend()
    axes[1, 1].set(title="ADMM iterations per task tick", xlabel="Time [s]", ylabel="Iterations")
    fig.suptitle("Safety, momentum, and QP timing diagnostics", fontsize=18, weight="bold")
    return _save_figure(fig, output / "safety_and_qp_diagnostics.png")


def _joint_torque_figure(trace: dict[str, np.ndarray], output: Path) -> dict[str, Any]:
    fig, axes = plt.subplots(2, 1, figsize=(14, 9), sharex=True, constrained_layout=True)
    palette = plt.cm.tab10(np.linspace(0.0, 0.85, 7))
    measured = trace["qpos"][:, 7:14]
    error = 1000.0 * (trace["reference_q"] - measured)
    for joint in range(7):
        axes[0].plot(trace["time"], error[:, joint], color=palette[joint], lw=1.0, label=f"J{joint + 1}")
        axes[1].plot(trace["time"], trace["torque"][:, joint], color=palette[joint], lw=1.0)
    axes[0].set_ylabel("Reference − measured [mrad]")
    axes[0].set_title("Low-level joint-servo tracking")
    axes[0].legend(ncol=7, loc="upper right")
    axes[1].set_ylabel("Torque [N·m]")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_title("Applied direct joint torques")
    fig.suptitle("Scenario 00 — 500 Hz torque execution", fontsize=18, weight="bold")
    return _save_figure(fig, output / "joint_tracking_and_torque.png")


def _camera(view: CameraView, trace: dict[str, np.ndarray], index: int) -> mujoco.MjvCamera:
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.fixedcamid = -1
    camera.trackbodyid = -1
    base = trace["base_qpos"][index, :3]
    flange = trace["flange_position"][index]
    camera.lookat[:] = 0.42 * base + 0.58 * flange
    if abs(view.elevation_deg) < 80.0:
        camera.lookat[2] += 0.10
    camera.azimuth = view.azimuth_deg
    camera.elevation = view.elevation_deg
    camera.distance = view.distance_m
    return camera


def _frame_axis_endpoints(position: np.ndarray, rotation: np.ndarray, length: float) -> np.ndarray:
    origin = np.asarray(position, dtype=np.float64).reshape(3)
    frame = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    return np.stack([np.stack((origin, origin + float(length) * frame[:, axis])) for axis in range(3)])


def _plot_coordinate_frame(
    axis: Any,
    position: np.ndarray,
    rotation: np.ndarray,
    length: float,
    alpha: float,
    linewidth: float,
) -> None:
    colors = ("#EF4444", "#22C55E", "#2563EB")
    for axis_index, endpoints in enumerate(_frame_axis_endpoints(position, rotation, length)):
        delta = endpoints[1] - endpoints[0]
        axis.quiver(
            *endpoints[0],
            *delta,
            color=colors[axis_index],
            alpha=alpha,
            arrow_length_ratio=0.22,
            linewidth=linewidth,
        )


def _append_coordinate_frame(
    scene: mujoco.MjvScene,
    position: np.ndarray,
    rotation: np.ndarray,
    length: float,
    alpha: float,
    radius: float,
) -> None:
    for axis_index, endpoints in enumerate(_frame_axis_endpoints(position, rotation, length)):
        if scene.ngeom >= scene.maxgeom:
            raise RuntimeError("MuJoCo scene geometry capacity exhausted")
        geometry = scene.geoms[scene.ngeom]
        color = AXIS_COLORS_RGBA[axis_index].copy()
        color[3] = alpha
        mujoco.mjv_initGeom(
            geometry,
            mujoco.mjtGeom.mjGEOM_ARROW,
            np.zeros(3),
            np.zeros(3),
            np.eye(3).reshape(-1),
            color.astype(np.float32),
        )
        mujoco.mjv_connector(
            geometry,
            mujoco.mjtGeom.mjGEOM_ARROW,
            float(radius),
            endpoints[0],
            endpoints[1],
        )
        scene.ngeom += 1


def _append_sphere(scene: mujoco.MjvScene, position: np.ndarray, radius: float, rgba: np.ndarray) -> None:
    if scene.ngeom >= scene.maxgeom:
        raise RuntimeError("MuJoCo scene geometry capacity exhausted")
    geometry = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geometry,
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.asarray([radius, 0.0, 0.0]),
        np.asarray(position, dtype=np.float64),
        np.eye(3).reshape(-1),
        np.asarray(rgba, dtype=np.float32),
    )
    scene.ngeom += 1


def _append_connector(scene: mujoco.MjvScene, start: np.ndarray, stop: np.ndarray, radius: float, rgba: np.ndarray) -> None:
    if scene.ngeom >= scene.maxgeom:
        raise RuntimeError("MuJoCo scene geometry capacity exhausted")
    geometry = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geometry,
        mujoco.mjtGeom.mjGEOM_CAPSULE,
        np.zeros(3),
        np.zeros(3),
        np.eye(3).reshape(-1),
        np.asarray(rgba, dtype=np.float32),
    )
    mujoco.mjv_connector(
        geometry,
        mujoco.mjtGeom.mjGEOM_CAPSULE,
        float(radius),
        np.asarray(start, dtype=np.float64),
        np.asarray(stop, dtype=np.float64),
    )
    scene.ngeom += 1


def _append_label(scene: mujoco.MjvScene, position: np.ndarray, label: str, rgba: np.ndarray) -> None:
    if scene.ngeom >= scene.maxgeom:
        raise RuntimeError("MuJoCo scene geometry capacity exhausted")
    geometry = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geometry,
        mujoco.mjtGeom.mjGEOM_LABEL,
        np.zeros(3),
        np.asarray(position, dtype=np.float64),
        np.eye(3).reshape(-1),
        np.asarray(rgba, dtype=np.float32),
    )
    geometry.label = str(label)
    scene.ngeom += 1


def _append_target_path(scene: mujoco.MjvScene, trace: dict[str, np.ndarray], view: CameraView) -> None:
    waypoints = trace["waypoint_points_static"]
    target_rotation = trace["target_rotation_static"]
    # The transition into W1 is blue-gray; the requested W1-W7 path is green.
    _append_connector(
        scene,
        trace["target_position"][0],
        waypoints[0],
        0.0015,
        np.asarray([0.38, 0.65, 1.00, 0.38]),
    )
    for index, waypoint in enumerate(waypoints):
        _append_sphere(scene, waypoint, 0.0075, np.asarray([1.00, 0.76, 0.08, 0.92]))
        _append_coordinate_frame(
            scene,
            waypoint,
            target_rotation,
            VIDEO_WAYPOINT_FRAME_AXIS_LENGTH_M,
            0.48,
            0.0022,
        )
        if index > 0:
            _append_connector(
                scene,
                waypoints[index - 1],
                waypoint,
                0.0020,
                np.asarray([0.10, 0.94, 0.48, 0.62]),
            )
        if view.key == "front":
            _append_label(
                scene,
                trace["waypoint_label_positions_static"][index],
                f"W{index + 1}",
                np.asarray([1.00, 0.93, 0.72, 1.0]),
            )


def _font(size: int) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(font_manager.findfont("DejaVu Sans"), size=size)


def _replace_background(frame: np.ndarray) -> np.ndarray:
    height, width, _ = frame.shape
    upper = np.asarray([24, 34, 49], dtype=np.float64)
    lower = np.asarray([6, 10, 18], dtype=np.float64)
    weights = np.linspace(0.0, 1.0, height)[:, None, None]
    gradient = ((1.0 - weights) * upper + weights * lower).astype(np.uint8)
    gradient = np.repeat(gradient, width, axis=1)
    result = frame.copy()
    mask = np.max(result, axis=2) <= 2
    result[mask] = gradient[mask]
    return result


def _draw_inset(draw: ImageDraw.ImageDraw, trace: dict[str, np.ndarray], index: int, width: int, height: int) -> None:
    inset_w, inset_h = 176, 130
    left, top = width - inset_w - 14, height - inset_h - 14
    draw.rounded_rectangle((left, top, left + inset_w, top + inset_h), radius=8, fill=(5, 12, 22, 215), outline=(100, 116, 139, 220), width=1)
    reference = trace["target_position"][:, 1:3]
    actual = trace["flange_position"][:, 1:3]
    values = np.vstack((reference, actual))
    lower = np.min(values, axis=0) - 0.015
    upper = np.max(values, axis=0) + 0.015

    def point(value: np.ndarray) -> tuple[int, int]:
        scaled = (value - lower) / np.maximum(upper - lower, 1e-12)
        return int(left + 10 + scaled[0] * (inset_w - 20)), int(top + inset_h - 10 - scaled[1] * (inset_h - 25))

    reference_points = [point(item) for item in reference[::40]]
    actual_step = max(1, (index + 1) // 150)
    actual_points = [point(item) for item in actual[: index + 1 : actual_step]]
    if len(reference_points) > 1:
        draw.line(reference_points, fill=(52, 211, 153, 220), width=2)
    if len(actual_points) > 1:
        draw.line(actual_points, fill=(96, 165, 250, 255), width=2)
    draw.ellipse((*np.subtract(point(reference[index]), (3, 3)), *np.add(point(reference[index]), (3, 3))), fill=(52, 211, 153, 255))
    draw.ellipse((*np.subtract(point(actual[index]), (3, 3)), *np.add(point(actual[index]), (3, 3))), fill=(96, 165, 250, 255))
    draw.text((left + 9, top + 5), "Y-Z tracking", font=_font(10), fill=(226, 232, 240, 255))


def _annotate(frame: np.ndarray, trace: dict[str, np.ndarray], index: int, view: CameraView, scenario: int) -> np.ndarray:
    image = Image.fromarray(_replace_background(frame)).convert("RGBA")
    overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    width, height = image.size
    draw.rectangle((0, 0, width, 56), fill=(3, 9, 18, 205))
    draw.text((16, 8), f"{view.title}  |  Scenario {scenario:02d}", font=_font(18), fill=(241, 245, 249, 255))
    error_mm = 1000.0 * float(trace["position_error"][index])
    clearance_mm = 1000.0 * float(trace["minimum_clearance"][index])
    time_s = float(trace["time"][index])
    status = f"t = {time_s:5.2f} s   error = {error_mm:6.3f} mm   clearance = {clearance_mm:6.1f} mm"
    draw.text((16, 32), status, font=_font(13), fill=(203, 213, 225, 255))
    draw.rectangle((0, height - 42, width, height), fill=(3, 9, 18, 205))
    draw.text((14, height - 36), "RGB=XYZ | long/faint: target | short/solid: EE | faint frames: W1-W7", font=_font(10), fill=(226, 232, 240, 255))
    draw.text((14, height - 19), "green path + gold points: W1-W7 | green dot: live target | red: obstacle", font=_font(9), fill=(148, 163, 184, 255))
    _draw_inset(draw, trace, index, width, height)
    image = Image.alpha_composite(image, overlay)
    return np.asarray(image.convert("RGB"))


def _render_setup(trace: dict[str, np.ndarray], width: int, height: int) -> tuple[mujoco.MjModel, mujoco.MjData, mujoco.Renderer]:
    spec = default_model_spec()
    model = spec.compile_model()
    model.geom_pos[geom_id(model, "workspace_obstacle_0")] = trace["obstacle_position"]
    model.geom_size[geom_id(model, "reference_marker_geom"), 0] = 0.018
    model.geom_contype[:] = 0
    model.geom_conaffinity[:] = 0
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=height, width=width)
    return model, data, renderer


def _render_at(model: mujoco.MjModel, data: mujoco.MjData, renderer: mujoco.Renderer, trace: dict[str, np.ndarray], index: int, view: CameraView, scenario: int) -> np.ndarray:
    data.qpos[:] = trace["qpos"][index]
    data.qvel[:] = trace["qvel"][index]
    data.time = float(trace["time"][index])
    data.mocap_pos[0] = trace["target_position"][index]
    mujoco.mj_forward(model, data)
    renderer.update_scene(data, camera=_camera(view, trace, index))
    _append_target_path(renderer.scene, trace, view)
    target_rotation = trace["target_rotation_static"]
    _append_coordinate_frame(
        renderer.scene,
        trace["target_position"][index],
        target_rotation,
        TARGET_FRAME_AXIS_LENGTH_M,
        0.62,
        0.0060,
    )
    _append_coordinate_frame(
        renderer.scene,
        trace["flange_position"][index],
        trace["flange_rotation"][index],
        END_EFFECTOR_FRAME_AXIS_LENGTH_M,
        1.0,
        0.0040,
    )
    return _annotate(renderer.render(), trace, index, view, scenario)


def _video_indices(trace: dict[str, np.ndarray], fps: int) -> np.ndarray:
    duration = float(trace["time"][-1])
    count = int(round(duration * fps))
    times = np.arange(count, dtype=np.float64) / float(fps)
    indices = np.searchsorted(trace["time"], times, side="left")
    return np.clip(indices, 0, len(trace["time"]) - 1)


def _encode_video(trace: dict[str, np.ndarray], view: CameraView, config: VideoConfig, output: Path) -> None:
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("ffmpeg is required to encode the five-view videos")
    command = [
        ffmpeg, "-y", "-loglevel", "error", "-f", "rawvideo", "-vcodec", "rawvideo",
        "-pix_fmt", "rgb24", "-s", f"{config.width}x{config.height}", "-r", str(config.fps),
        "-i", "-", "-an", "-c:v", "libx264", "-preset", "fast", "-crf", str(config.crf),
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(output),
    ]
    flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.PIPE, creationflags=flags)
    assert process.stdin is not None
    model, data, renderer = _render_setup(trace, config.width, config.height)
    try:
        for index in _video_indices(trace, config.fps):
            frame = _render_at(model, data, renderer, trace, int(index), view, config.scenario_index)
            process.stdin.write(np.ascontiguousarray(frame).tobytes())
    finally:
        renderer.close()
        process.stdin.close()
    assert process.stderr is not None
    error = process.stderr.read().decode("utf-8", errors="replace")
    return_code = process.wait()
    if return_code != 0:
        raise RuntimeError(f"ffmpeg failed for {view.key}: {error}")


def _probe_video(path: Path) -> dict[str, Any]:
    ffprobe = shutil.which("ffprobe")
    if ffprobe is None:
        raise RuntimeError("ffprobe is required to validate videos")
    command = [
        ffprobe, "-v", "error", "-count_frames", "-select_streams", "v:0",
        "-show_entries", "stream=width,height,avg_frame_rate,nb_read_frames:format=duration",
        "-of", "json", str(path),
    ]
    flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    result = subprocess.run(command, check=True, capture_output=True, text=True, creationflags=flags)
    payload = json.loads(result.stdout)
    stream = payload["streams"][0]
    numerator, denominator = (float(value) for value in stream["avg_frame_rate"].split("/"))
    return {
        "path": str(path.relative_to(PROJECT_ROOT)),
        "sha256": _sha256(path),
        "bytes": path.stat().st_size,
        "width": int(stream["width"]),
        "height": int(stream["height"]),
        "fps": numerator / denominator,
        "frames": int(stream["nb_read_frames"]),
        "duration_s": float(payload["format"]["duration"]),
    }


def _preview(trace: dict[str, np.ndarray], config: VideoConfig, output: Path) -> dict[str, Any]:
    index = len(trace["time"]) // 2
    tiles = []
    model, data, renderer = _render_setup(trace, config.width, config.height)
    try:
        for view in VIEWS:
            frame = _render_at(model, data, renderer, trace, index, view, config.scenario_index)
            tiles.append(Image.fromarray(frame).resize((480, 360), Image.Resampling.LANCZOS))
    finally:
        renderer.close()
    canvas = Image.new("RGB", (1440, 720), (8, 13, 23))
    for tile_index, tile in enumerate(tiles):
        canvas.paste(tile, ((tile_index % 3) * 480, (tile_index // 3) * 360))
    draw = ImageDraw.Draw(canvas)
    draw.rounded_rectangle((980, 410, 1410, 670), radius=14, fill=(15, 23, 42), outline=(71, 85, 105), width=2)
    draw.text((1010, 445), "Five synchronized views", font=_font(24), fill=(241, 245, 249))
    draw.text((1010, 500), "Same state, target, obstacle,\nand timestamp in every tile.", font=_font(17), fill=(203, 213, 225), spacing=8)
    draw.text((1010, 605), f"Snapshot: t = {trace['time'][index]:.2f} s", font=_font(16), fill=(94, 234, 212))
    canvas.save(output)
    return {"path": str(output.relative_to(PROJECT_ROOT)), "sha256": _sha256(output), "width": 1440, "height": 720, "bytes": output.stat().st_size}


def generate_visualizations(config: VideoConfig, output_dir: Path) -> dict[str, Any]:
    _style()
    output_dir = Path(output_dir).resolve()
    video_dir = output_dir / "videos"
    output_dir.mkdir(parents=True, exist_ok=True)
    video_dir.mkdir(parents=True, exist_ok=True)
    metrics_path = PROJECT_ROOT / "output" / "migration_metrics.json"
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    trace_paths = [PROJECT_ROOT / item["trace"] for item in metrics["scenarios"]]
    traces = [_load_npz(path) for path in trace_paths]
    for index, trace in enumerate(traces):
        trace["target_rotation_static"] = np.asarray(
            metrics["scenarios"][index]["target_contract"]["target_rotation_world"],
            dtype=np.float64,
        )
        waypoints = np.asarray(
            metrics["scenarios"][index]["target_contract"]["waypoint_points_m"],
            dtype=np.float64,
        )
        trace["waypoint_points_static"] = waypoints
        centroid = np.mean(waypoints, axis=0)
        radial = waypoints - centroid
        radial /= np.maximum(np.linalg.norm(radial, axis=1, keepdims=True), 1e-12)
        trace["waypoint_label_positions_static"] = waypoints + 0.035 * radial + np.asarray([0.0, 0.0, 0.018])
        trace["waypoint_label_positions_static"][0] = waypoints[0] + np.asarray([0.0, -0.030, 0.060])
        trace["waypoint_label_positions_static"][-1] = waypoints[-1] + np.asarray([0.0, -0.030, -0.060])
    if not 0 <= config.scenario_index < len(traces):
        raise ValueError("scenario index is outside the generated suite")
    figures = [
        _tracking_paths(traces, metrics, output_dir),
        _error_curves(traces, output_dir),
        _base_drift_figure(traces, output_dir),
        _safety_qp_figure(traces, output_dir),
        _joint_torque_figure(traces[config.scenario_index], output_dir),
    ]
    selected = traces[config.scenario_index]
    videos = []
    for view in VIEWS:
        path = video_dir / f"scenario_{config.scenario_index:02d}_{view.key}.mp4"
        _encode_video(selected, view, config, path)
        videos.append(_probe_video(path))
    preview = _preview(selected, config, output_dir / "five_view_preview.png")
    expected_frames = len(_video_indices(selected, config.fps))
    expected_duration = expected_frames / config.fps
    checks = {
        "five_distinct_views": len(videos) == 5 and len({item["path"] for item in videos}) == 5,
        "all_videos_h264_readable": all(item["bytes"] > 10000 for item in videos),
        "all_video_dimensions_match": all(item["width"] == config.width and item["height"] == config.height for item in videos),
        "all_video_frame_counts_match": all(item["frames"] == expected_frames for item in videos),
        "all_video_durations_match": all(abs(item["duration_s"] - expected_duration) <= 1.0 / config.fps for item in videos),
        "all_video_rates_match": all(abs(item["fps"] - config.fps) <= 1e-9 for item in videos),
        "five_static_figures_nonempty": len(figures) == 5 and all(item["bytes"] > 20000 for item in figures),
        "preview_is_1440x720": preview["width"] == 1440 and preview["height"] == 720,
        "source_suite_contains_five_scenarios": len(traces) == 5,
    }
    report = {
        "passed": all(checks.values()),
        "checks": checks,
        "video_config": asdict(config),
        "source_metrics": str(metrics_path.relative_to(PROJECT_ROOT)),
        "source_metrics_sha256": _sha256(metrics_path),
        "source_trace": str(trace_paths[config.scenario_index].relative_to(PROJECT_ROOT)),
        "source_trace_sha256": _sha256(trace_paths[config.scenario_index]),
        "views": [asdict(view) for view in VIEWS],
        "coordinate_frames": {
            "axis_color_order": ["x_red", "y_green", "z_blue"],
            "target_axis_length_m": TARGET_FRAME_AXIS_LENGTH_M,
            "end_effector_axis_length_m": END_EFFECTOR_FRAME_AXIS_LENGTH_M,
            "waypoint_axis_length_m": WAYPOINT_FRAME_AXIS_LENGTH_M,
            "video_waypoint_axis_length_m": VIDEO_WAYPOINT_FRAME_AXIS_LENGTH_M,
            "target_style": "long_translucent_arrows",
            "end_effector_style": "short_opaque_arrows",
            "video_frames_rendered": ["live_target_point_frame", "live_flexiv_flange_frame", "W1_to_W7_target_frames"],
            "path_figure_frames_rendered": ["W1_to_W7_target_frames", "final_target_frame", "final_flexiv_flange_frame"],
        },
        "video_target_path": {
            "geometry": "W1_to_W7_capsule_polyline",
            "waypoint_count": 7,
            "waypoint_markers": "gold_spheres",
            "waypoint_labels": "W1_to_W7_in_front_view",
            "transition_to_W1": "blue_gray_connector",
            "path_color": "green",
        },
        "figures": figures,
        "preview": preview,
        "videos": videos,
    }
    manifest_path = output_dir / "visualization_manifest.json"
    manifest_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    if not report["passed"]:
        raise RuntimeError("visualization validation failed; inspect visualization_manifest.json")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", type=int, default=0)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--crf", type=int, default=20)
    parser.add_argument("--output-dir", type=Path, default=PROJECT_ROOT / "output" / "visualization")
    args = parser.parse_args()
    report = generate_visualizations(VideoConfig(args.scenario, args.width, args.height, args.fps, args.crf), args.output_dir)
    print(json.dumps({"passed": report["passed"], "figures": len(report["figures"]), "videos": len(report["videos"])}, indent=2))


if __name__ == "__main__":
    main()
