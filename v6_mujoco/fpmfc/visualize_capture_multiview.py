"""Render five synchronized capture views with live joint kinematics."""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
from dataclasses import asdict
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
from PIL import Image, ImageDraw

from v6_mujoco.model import PROJECT_ROOT, default_model_spec
from v6_mujoco.visualize import CameraView, _font

from .visualize_capture import _load_trace, _render_frame, _sha256, _video_indices


CAPTURE_VIEWS = (
    CameraView("isometric", "Isometric", 137.0, -24.0, 1.85),
    CameraView("front", "Front", 180.0, -8.0, 1.85),
    CameraView("right", "Right side", 90.0, -10.0, 1.85),
    CameraView("top", "Top", 180.0, -89.0, 1.75),
    CameraView("rear", "Rear", 0.0, -14.0, 1.85),
)

JOINT_COLORS = (
    (0, 114, 178),
    (213, 94, 0),
    (0, 158, 115),
    (204, 121, 167),
    (230, 159, 0),
    (86, 180, 233),
    (240, 228, 66),
)


def _joint_kinematics(
    trace: dict[str, np.ndarray],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    time = np.asarray(trace["time"], dtype=np.float64)
    if time.ndim != 1 or len(time) < 3 or not np.all(np.diff(time) > 0.0):
        raise ValueError("trace time must be a strictly increasing vector")
    qpos = np.asarray(trace["qpos"], dtype=np.float64)
    qvel = np.asarray(trace["qvel"], dtype=np.float64)
    if qpos.shape != (len(time), 14) or qvel.shape != (len(time), 13):
        raise ValueError("expected free-base qpos[14] and qvel[13] trace arrays")
    joint_position = qpos[:, 7:14]
    joint_velocity = qvel[:, 6:13]
    joint_acceleration = np.gradient(joint_velocity, time, axis=0, edge_order=2)
    if not np.all(np.isfinite(joint_acceleration)):
        raise ValueError("numerically differentiated joint acceleration is non-finite")
    return joint_position, joint_velocity, joint_acceleration


def _start_encoder(
    path: Path,
    width: int,
    height: int,
    fps: int,
    crf: int,
) -> subprocess.Popen[bytes]:
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("ffmpeg is required")
    command = [
        ffmpeg,
        "-y",
        "-loglevel",
        "error",
        "-f",
        "rawvideo",
        "-vcodec",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-s",
        f"{width}x{height}",
        "-r",
        str(fps),
        "-i",
        "-",
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        "fast",
        "-crf",
        str(crf),
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(path),
    ]
    flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    process = subprocess.Popen(
        command,
        stdin=subprocess.PIPE,
        stderr=subprocess.PIPE,
        creationflags=flags,
    )
    return process


def _finish_encoder(process: subprocess.Popen[bytes], label: str) -> None:
    if process.stdin is not None and not process.stdin.closed:
        process.stdin.close()
    error = ""
    if process.stderr is not None:
        error = process.stderr.read().decode("utf-8", errors="replace")
    return_code = process.wait()
    if return_code != 0:
        raise RuntimeError(f"ffmpeg failed for {label}: {error}")


def _write_frame(process: subprocess.Popen[bytes], frame: np.ndarray) -> None:
    if process.stdin is None:
        raise RuntimeError("ffmpeg stdin is unavailable")
    process.stdin.write(np.ascontiguousarray(frame, dtype=np.uint8).tobytes())


def _probe_video(path: Path) -> dict[str, Any]:
    ffprobe = shutil.which("ffprobe")
    if ffprobe is None:
        raise RuntimeError("ffprobe is required")
    flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    result = subprocess.run(
        [
            ffprobe,
            "-v",
            "error",
            "-count_frames",
            "-select_streams",
            "v:0",
            "-show_entries",
            "stream=width,height,avg_frame_rate,nb_read_frames:format=duration",
            "-of",
            "json",
            str(path),
        ],
        check=True,
        capture_output=True,
        text=True,
        creationflags=flags,
    )
    payload = json.loads(result.stdout)
    stream = payload["streams"][0]
    numerator, denominator = (float(item) for item in stream["avg_frame_rate"].split("/"))
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


def _decode_video(path: Path) -> bool:
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("ffmpeg is required")
    flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    result = subprocess.run(
        [ffmpeg, "-v", "error", "-i", str(path), "-map", "0:v:0", "-f", "null", "-"],
        capture_output=True,
        creationflags=flags,
    )
    return result.returncode == 0 and not result.stderr


def _curve_panel_base(
    time: np.ndarray,
    values: np.ndarray,
    width: int,
    height: int,
    title: str,
    unit: str,
) -> tuple[Image.Image, float, float, tuple[int, int, int, int]]:
    background = (8, 13, 23)
    image = Image.new("RGB", (width, height), background)
    draw = ImageDraw.Draw(image)
    left, top, right, bottom = 64, 54, width - 18, height - 48
    lo = float(np.min(values))
    hi = float(np.max(values))
    if not np.isfinite(lo) or not np.isfinite(hi):
        raise ValueError(f"non-finite values in {title}")
    span = hi - lo
    if span < 1.0e-9:
        span = max(abs(lo), 1.0) * 0.2
    pad = 0.08 * span
    lo -= pad
    hi += pad

    draw.text((12, 8), title, font=_font(17), fill=(241, 245, 249))
    draw.text((12, 32), unit, font=_font(11), fill=(148, 163, 184))
    draw.rectangle((left, top, right, bottom), outline=(71, 85, 105), width=1)

    for tick in range(5):
        ratio = tick / 4.0
        y = int(round(bottom - ratio * (bottom - top)))
        value = lo + ratio * (hi - lo)
        draw.line((left, y, right, y), fill=(30, 41, 59), width=1)
        draw.text((4, y - 6), f"{value: .2f}", font=_font(9), fill=(148, 163, 184))
    for tick in range(5):
        ratio = tick / 4.0
        x = int(round(left + ratio * (right - left)))
        draw.line((x, top, x, bottom), fill=(30, 41, 59), width=1)
        value = float(time[0] + ratio * (time[-1] - time[0]))
        draw.text((x - 11, bottom + 8), f"{value:.1f}", font=_font(9), fill=(148, 163, 184))
    draw.text((right - 42, bottom + 25), "t [s]", font=_font(10), fill=(203, 213, 225))

    sample_count = min(len(time), max(240, 2 * (right - left)))
    sample = np.unique(np.linspace(0, len(time) - 1, sample_count).astype(int))
    x_values = left + (time[sample] - time[0]) / (time[-1] - time[0]) * (right - left)
    for joint in range(7):
        y_values = bottom - (values[sample, joint] - lo) / (hi - lo) * (bottom - top)
        points = [(int(round(x)), int(round(y))) for x, y in zip(x_values, y_values)]
        draw.line(points, fill=JOINT_COLORS[joint], width=2)

    legend_x = 188
    for joint in range(7):
        x = legend_x + joint * 59
        draw.line((x, 24, x + 13, 24), fill=JOINT_COLORS[joint], width=3)
        draw.text((x + 17, 17), f"J{joint + 1}", font=_font(9), fill=(203, 213, 225))
    return image, lo, hi, (left, top, right, bottom)


def _curve_panel_frame(
    base: Image.Image,
    time: np.ndarray,
    values: np.ndarray,
    index: int,
    lo: float,
    hi: float,
    plot_box: tuple[int, int, int, int],
) -> Image.Image:
    image = base.copy()
    draw = ImageDraw.Draw(image)
    left, top, right, bottom = plot_box
    ratio = (float(time[index]) - float(time[0])) / (float(time[-1]) - float(time[0]))
    x = int(round(left + ratio * (right - left)))
    draw.line((x, top, x, bottom), fill=(241, 245, 249), width=2)
    for joint in range(7):
        y = int(round(bottom - (float(values[index, joint]) - lo) / (hi - lo) * (bottom - top)))
        draw.ellipse((x - 3, y - 3, x + 3, y + 3), fill=JOINT_COLORS[joint], outline=(8, 13, 23))
    draw.text((left + 6, top + 5), f"t={float(time[index]):.3f} s", font=_font(10), fill=(241, 245, 249))
    return image


def _status_tile(
    width: int,
    height: int,
    time: np.ndarray,
    index: int,
    q: np.ndarray,
    dq: np.ndarray,
    ddq: np.ndarray,
    trace: dict[str, np.ndarray],
) -> Image.Image:
    image = Image.new("RGB", (width, height), (8, 13, 23))
    draw = ImageDraw.Draw(image)
    draw.text((24, 22), "Synchronized telemetry", font=_font(23), fill=(241, 245, 249))
    draw.text(
        (24, 60),
        f"Frame time: {float(time[index]):.3f} s   |   trace sample: {index + 1}/{len(time)}",
        font=_font(13),
        fill=(94, 234, 212),
    )
    draw.text((24, 92), "Five camera tiles and all three plots use this exact sample.", font=_font(12), fill=(203, 213, 225))
    draw.text((24, 125), f"max |q|     {float(np.max(np.abs(q[index]))):8.4f} rad", font=_font(13), fill=(226, 232, 240))
    draw.text((24, 150), f"max |dq|    {float(np.max(np.abs(dq[index]))):8.4f} rad/s", font=_font(13), fill=(226, 232, 240))
    draw.text((24, 175), f"max |ddq|   {float(np.max(np.abs(ddq[index]))):8.4f} rad/s^2", font=_font(13), fill=(226, 232, 240))
    draw.text(
        (24, 210),
        f"EE error {1000.0 * float(trace['position_error_m'][index]):.4f} mm   |   clearance {1000.0 * float(trace['minimum_clearance_m'][index]):.2f} mm",
        font=_font(12),
        fill=(203, 213, 225),
    )
    draw.text((24, 246), "Joint colors", font=_font(13), fill=(241, 245, 249))
    for joint in range(7):
        column = joint % 4
        row = joint // 4
        x = 24 + column * 145
        y = 279 + row * 28
        draw.line((x, y, x + 24, y), fill=JOINT_COLORS[joint], width=4)
        draw.text((x + 34, y - 8), f"J{joint + 1}", font=_font(12), fill=(203, 213, 225))
    draw.text((390, 324), "ddq = d(dq)/dt, central difference", font=_font(9), fill=(148, 163, 184))
    return image


def _save_static_joint_figure(
    time: np.ndarray,
    q: np.ndarray,
    dq: np.ndarray,
    ddq: np.ndarray,
    output_dir: Path,
) -> list[dict[str, Any]]:
    plt.style.use("seaborn-v0_8-whitegrid")
    colors = [tuple(channel / 255.0 for channel in color) for color in JOINT_COLORS]
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True, constrained_layout=True)
    signals = ((q, "Joint position [rad]"), (dq, "Joint velocity [rad/s]"), (ddq, "Joint acceleration [rad/s$^2$]"))
    for axis, (values, label) in zip(axes, signals):
        for joint in range(7):
            axis.plot(time, values[:, joint], color=colors[joint], linewidth=1.1, label=f"J{joint + 1}")
        axis.set_ylabel(label)
        axis.grid(True, alpha=0.28)
    axes[0].legend(ncol=7, loc="upper center", bbox_to_anchor=(0.5, 1.24), frameon=False)
    axes[-1].set_xlabel("Time [s]")
    fig.suptitle("Selected capture — synchronized joint kinematics", fontsize=16)
    outputs = []
    for suffix in ("png", "pdf"):
        path = output_dir / f"joint_position_velocity_acceleration.{suffix}"
        fig.savefig(path, dpi=180, bbox_inches="tight")
        outputs.append(
            {
                "path": str(path.relative_to(PROJECT_ROOT)),
                "sha256": _sha256(path),
                "bytes": path.stat().st_size,
            }
        )
    plt.close(fig)
    return outputs


def render_multiview_capture(
    trace_path: Path,
    metrics_path: Path,
    output_dir: Path,
    width: int = 640,
    height: int = 360,
    fps: int = 30,
    crf: int = 20,
) -> dict[str, Any]:
    trace_path = trace_path.resolve()
    metrics_path = metrics_path.resolve()
    output_dir = output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    trace = _load_trace(trace_path)
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    time = np.asarray(trace["time"], dtype=np.float64)
    q, dq, ddq = _joint_kinematics(trace)
    indices = _video_indices(time, fps)

    target_center = np.asarray(metrics["target"]["initial_center_position_world_m"], dtype=np.float64)
    target_rotation = np.asarray(metrics["target"]["initial_rotation_world"], dtype=np.float64)
    grasp_offset_m = float(np.linalg.norm(metrics["target"]["grasp_offset_body_m"]))
    target_side_length_m = float(metrics["target"].get("side_length_m", 2.0 * grasp_offset_m))
    terminal_shape_rad = float(metrics["candidate"]["terminal_arm_angle_rad"])
    sample = np.unique(np.linspace(0, len(time) - 1, 120).astype(int))
    path_points = trace["target_grasp_position"][sample]

    spec = default_model_spec()
    model = spec.compile_model()
    target_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "tumbling_target")
    target_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "tumbling_target_geom")
    if target_body < 0 or target_geom < 0:
        raise RuntimeError("MuJoCo target cube is missing")
    target_mocap_id = int(model.body_mocapid[target_body])
    if target_mocap_id < 0:
        raise RuntimeError("MuJoCo target cube must be a mocap body")
    model.geom_size[target_geom, :3] = 0.5 * target_side_length_m
    model.geom_contype[:] = 0
    model.geom_conaffinity[:] = 0
    model.vis.global_.offwidth = max(int(model.vis.global_.offwidth), int(width))
    model.vis.global_.offheight = max(int(model.vis.global_.offheight), int(height))
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=height, width=width)

    panel_specs = (
        (q, "Joint position", "q [rad]"),
        (dq, "Joint velocity", "dq [rad/s]"),
        (ddq, "Joint acceleration", "ddq [rad/s^2]"),
    )
    panel_bases = [
        _curve_panel_base(time, values, width, height, title, unit)
        for values, title, unit in panel_specs
    ]

    view_paths = {view.key: output_dir / f"selected_capture_{view.key}.mp4" for view in CAPTURE_VIEWS}
    composite_path = output_dir / "selected_capture_five_view_joint_kinematics.mp4"
    view_encoders = {
        view.key: _start_encoder(view_paths[view.key], width, height, fps, crf)
        for view in CAPTURE_VIEWS
    }
    composite_width = 3 * width
    composite_height = 3 * height
    composite_encoder = _start_encoder(composite_path, composite_width, composite_height, fps, crf)
    preview_path = output_dir / "five_view_joint_kinematics_preview.png"
    preview_written = False
    encoding_error = None
    try:
        for output_index, trace_index_value in enumerate(indices):
            trace_index = int(trace_index_value)
            frames = []
            for view in CAPTURE_VIEWS:
                frame = _render_frame(
                    model,
                    data,
                    renderer,
                    trace,
                    trace_index,
                    target_center,
                    target_rotation,
                    target_side_length_m,
                    grasp_offset_m,
                    target_mocap_id,
                    path_points,
                    terminal_shape_rad,
                    view,
                )
                frames.append(Image.fromarray(frame))
                _write_frame(view_encoders[view.key], frame)

            curve_frames = [
                _curve_panel_frame(base, time, values, trace_index, lo, hi, plot_box)
                for (base, lo, hi, plot_box), (values, _, _) in zip(panel_bases, panel_specs)
            ]
            composite = Image.new("RGB", (composite_width, composite_height), (8, 13, 23))
            for tile_index, frame in enumerate(frames[:3]):
                composite.paste(frame, (tile_index * width, 0))
            composite.paste(frames[3], (0, height))
            composite.paste(frames[4], (width, height))
            composite.paste(_status_tile(width, height, time, trace_index, q, dq, ddq, trace), (2 * width, height))
            for panel_index, panel in enumerate(curve_frames):
                composite.paste(panel, (panel_index * width, 2 * height))
            composite_array = np.asarray(composite)
            _write_frame(composite_encoder, composite_array)
            if output_index == len(indices) // 2:
                composite.save(preview_path)
                preview_written = True
    except Exception as exc:
        encoding_error = exc
    finally:
        renderer.close()
        encoder_errors = []
        for key, process in view_encoders.items():
            try:
                _finish_encoder(process, key)
            except Exception as exc:
                encoder_errors.append(exc)
        try:
            _finish_encoder(composite_encoder, "composite")
        except Exception as exc:
            encoder_errors.append(exc)
        if encoding_error is not None:
            raise encoding_error
        if encoder_errors:
            raise encoder_errors[0]

    if not preview_written:
        raise RuntimeError("preview frame was not written")
    static_figures = _save_static_joint_figure(time, q, dq, ddq, output_dir)
    view_videos = []
    for view in CAPTURE_VIEWS:
        item = _probe_video(view_paths[view.key])
        item["key"] = view.key
        item["title"] = view.title
        item["decoded"] = _decode_video(view_paths[view.key])
        view_videos.append(item)
    composite_video = _probe_video(composite_path)
    composite_video["decoded"] = _decode_video(composite_path)
    expected_frames = len(indices)
    expected_duration = expected_frames / float(fps)
    preview = {
        "path": str(preview_path.relative_to(PROJECT_ROOT)),
        "sha256": _sha256(preview_path),
        "bytes": preview_path.stat().st_size,
        "width": composite_width,
        "height": composite_height,
    }
    checks = {
        "source_trace_matches_metrics": metrics["trace"]["sha256"] == _sha256(trace_path),
        "five_camera_definitions_unique": len({(view.azimuth_deg, view.elevation_deg) for view in CAPTURE_VIEWS}) == 5,
        "five_view_videos_present": len(view_videos) == 5 and all(item["bytes"] > 10_000 for item in view_videos),
        "all_view_videos_decode": all(item["decoded"] for item in view_videos),
        "all_view_dimensions_match": all(item["width"] == width and item["height"] == height for item in view_videos),
        "all_view_frames_match": all(item["frames"] == expected_frames for item in view_videos),
        "all_view_durations_match": all(abs(item["duration_s"] - expected_duration) <= 1.0 / fps for item in view_videos),
        "composite_video_decodes": bool(composite_video["decoded"]),
        "composite_dimensions_match": composite_video["width"] == composite_width and composite_video["height"] == composite_height,
        "composite_frames_match": composite_video["frames"] == expected_frames,
        "composite_duration_matches": abs(composite_video["duration_s"] - expected_duration) <= 1.0 / fps,
        "joint_arrays_are_synchronized": q.shape == dq.shape == ddq.shape == (len(time), 7),
        "joint_acceleration_is_finite": bool(np.all(np.isfinite(ddq))),
        "static_curves_nonempty": len(static_figures) == 2 and all(item["bytes"] > 20_000 for item in static_figures),
        "preview_dimensions_match": preview["width"] == composite_width and preview["height"] == composite_height,
    }
    report = {
        "material_passport": {
            "origin_skill": "visualize",
            "origin_mode": "verified-trace-five-view-render",
            "verification_status": "VERIFIED",
            "version_label": "fpmfc_five_view_joint_kinematics_v1",
        },
        "source_trace": str(trace_path.relative_to(PROJECT_ROOT)),
        "source_trace_sha256": _sha256(trace_path),
        "source_metrics": str(metrics_path.relative_to(PROJECT_ROOT)),
        "source_metrics_sha256": _sha256(metrics_path),
        "video_config": {
            "tile_width": width,
            "tile_height": height,
            "composite_width": composite_width,
            "composite_height": composite_height,
            "fps": fps,
            "crf": crf,
            "expected_frames": expected_frames,
            "expected_duration_s": expected_duration,
        },
        "views": [asdict(view) for view in CAPTURE_VIEWS],
        "joint_signals": {
            "position_source": "qpos[:, 7:14]",
            "velocity_source": "qvel[:, 6:13]",
            "acceleration_source": "numpy.gradient(joint_velocity, time, edge_order=2)",
            "sample_count": len(time),
            "sample_period_s": float(np.median(np.diff(time))),
            "units": {"position": "rad", "velocity": "rad/s", "acceleration": "rad/s^2"},
        },
        "view_videos": view_videos,
        "composite_video": composite_video,
        "static_joint_figures": static_figures,
        "preview": preview,
        "checks": checks,
        "passed": all(checks.values()),
    }
    manifest_path = output_dir / "five_view_joint_kinematics_manifest.json"
    manifest_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    if not report["passed"]:
        raise RuntimeError("five-view visualization validation failed")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--metrics", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=360)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--crf", type=int, default=20)
    args = parser.parse_args()
    if min(args.width, args.height, args.fps) <= 0:
        raise ValueError("width, height, and fps must be positive")
    report = render_multiview_capture(
        args.trace,
        args.metrics,
        args.output_dir,
        width=args.width,
        height=args.height,
        fps=args.fps,
        crf=args.crf,
    )
    print(
        json.dumps(
            {
                "passed": report["passed"],
                "view_videos": len(report["view_videos"]),
                "composite_video": report["composite_video"]["path"],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
