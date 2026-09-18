"""Render a verified FPMFC capture trace as an annotated MuJoCo video."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
from pathlib import Path
from typing import Optional

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from v6_mujoco.model import PROJECT_ROOT, default_model_spec
from v6_mujoco.visualize import (
    CameraView,
    _append_connector,
    _append_coordinate_frame,
    _append_sphere,
    _font,
    _replace_background,
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_trace(path: Path) -> dict[str, np.ndarray]:
    with np.load(path, allow_pickle=False) as archive:
        return {key: archive[key] for key in archive.files}


def _camera(
    trace: dict[str, np.ndarray],
    index: int,
    view: Optional[CameraView] = None,
) -> mujoco.MjvCamera:
    if view is None:
        view = CameraView("isometric", "Isometric", 137.0, -24.0, 1.85)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.fixedcamid = -1
    camera.trackbodyid = -1
    camera.lookat[:] = 0.38 * trace["base_qpos"][index, :3] + 0.62 * trace["flange_position"][index]
    camera.lookat[2] += 0.08
    camera.azimuth = view.azimuth_deg
    camera.elevation = view.elevation_deg
    camera.distance = view.distance_m
    return camera


def _video_indices(time: np.ndarray, fps: int) -> np.ndarray:
    count = int(round(float(time[-1]) * fps)) + 1
    desired = np.arange(count, dtype=np.float64) / float(fps)
    return np.clip(np.searchsorted(time, desired, side="left"), 0, len(time) - 1)


def _annotate(
    frame: np.ndarray,
    trace: dict[str, np.ndarray],
    index: int,
    terminal_shape_rad: float,
    target_side_length_m: float,
    grasp_offset_m: float,
    view_title: Optional[str] = None,
) -> np.ndarray:
    image = Image.fromarray(_replace_background(frame)).convert("RGBA")
    overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    width, height = image.size
    compact = height < 500
    top_height = 56 if compact else 76
    bottom_height = 25 if compact else 36
    draw.rectangle((0, 0, width, top_height), fill=(3, 9, 18, 215))
    heading = "Adaptive FPMFC | complete pre-contact capture"
    if view_title:
        heading = f"{view_title} | {heading}"
    draw.text(
        (12 if compact else 16, 4 if compact else 8),
        heading,
        font=_font(13 if compact else 17),
        fill=(241, 245, 249, 255),
    )
    base_omega = float(np.linalg.norm(trace["base_twist"][index, 3:]))
    line = (
        f"t={float(trace['time'][index]):5.2f} s   EE={1000.0 * float(trace['position_error_m'][index]):7.4f} mm   "
        f"base w={base_omega:7.5f} rad/s"
    )
    draw.text(
        (12 if compact else 16, 22 if compact else 34),
        line,
        font=_font(9 if compact else 12),
        fill=(203, 213, 225, 255),
    )
    shape_error_deg = abs(float(trace["arm_angle_rad"][index]) - terminal_shape_rad) * 180.0 / np.pi
    line2 = f"shape-to-terminal={shape_error_deg:7.3f} deg   clearance={1000.0 * float(trace['minimum_clearance_m'][index]):7.2f} mm"
    draw.text(
        (12 if compact else 16, 37 if compact else 53),
        line2,
        font=_font(9 if compact else 12),
        fill=(203, 213, 225, 255),
    )
    draw.rectangle((0, height - bottom_height, width, height), fill=(3, 9, 18, 215))
    footer = "RGB=XYZ | translucent: grasp frame | solid: flange"
    if not compact:
        footer = "RGB = XYZ | long/translucent: tumbling grasp frame | short/solid: Flexiv flange"
    draw.text(
        (10 if compact else 14, height - (18 if compact else 29)),
        footer,
        font=_font(8 if compact else 10),
        fill=(226, 232, 240, 255),
    )
    target_line = (
        f"gold: target center | green: grasp point/path | target cube: {target_side_length_m:.2f} m | "
        f"grasp offset: {grasp_offset_m:.2f} m"
    )
    if not compact:
        draw.text((14, height - 14), target_line, font=_font(9), fill=(148, 163, 184, 255))
    return np.asarray(Image.alpha_composite(image, overlay).convert("RGB"))


def _render_frame(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    renderer: mujoco.Renderer,
    trace: dict[str, np.ndarray],
    index: int,
    target_center_initial: np.ndarray,
    target_rotation_initial: np.ndarray,
    target_side_length_m: float,
    grasp_offset_m: float,
    target_mocap_id: int,
    path_points: np.ndarray,
    terminal_shape_rad: float,
    view: Optional[CameraView] = None,
) -> np.ndarray:
    data.qpos[:] = trace["qpos"][index]
    data.qvel[:] = trace["qvel"][index]
    data.time = float(trace["time"][index])
    target_center = (
        trace["target_center_position"][index]
        if "target_center_position" in trace
        else target_center_initial
    )
    target_rotation = (
        trace["target_center_rotation"][index]
        if "target_center_rotation" in trace
        else target_rotation_initial
    )
    target_grasp = (
        trace["target_grasp_position"][index]
        if "target_grasp_position" in trace
        else trace["desired_position"][index]
    )
    target_grasp_rotation = (
        trace["target_grasp_rotation"][index]
        if "target_grasp_rotation" in trace
        else trace["desired_rotation"][index]
    )
    if model.nmocap:
        data.mocap_pos[0] = target_grasp
        mujoco.mju_mat2Quat(data.mocap_quat[0], target_grasp_rotation.reshape(-1))
        data.mocap_pos[target_mocap_id] = target_center
        mujoco.mju_mat2Quat(
            data.mocap_quat[target_mocap_id], target_rotation.reshape(-1)
        )
    mujoco.mj_forward(model, data)
    renderer.update_scene(data, camera=_camera(trace, index, view))

    base_center = trace["base_qpos"][index, :3]
    for start, stop in zip(path_points[:-1], path_points[1:]):
        _append_connector(renderer.scene, start, stop, 0.0017, np.asarray([0.10, 0.94, 0.48, 0.45]))
    _append_sphere(renderer.scene, base_center, 0.014, np.asarray([0.05, 0.07, 0.10, 0.98]))
    _append_connector(
        renderer.scene,
        base_center,
        target_center,
        0.003,
        np.asarray([0.08, 0.72, 0.70, 0.72]),
    )
    _append_sphere(renderer.scene, target_center, 0.018, np.asarray([1.00, 0.76, 0.08, 0.96]))
    _append_connector(renderer.scene, target_center, target_grasp, 0.004, np.asarray([1.00, 0.76, 0.08, 0.62]))
    _append_sphere(renderer.scene, target_grasp, 0.012, np.asarray([0.10, 0.94, 0.48, 0.98]))
    _append_coordinate_frame(renderer.scene, target_grasp, target_grasp_rotation, 0.13, 0.62, 0.005)
    _append_coordinate_frame(
        renderer.scene,
        trace["flange_position"][index],
        trace["flange_rotation"][index],
        0.085,
        1.0,
        0.0035,
    )
    return _annotate(
        renderer.render(),
        trace,
        index,
        terminal_shape_rad,
        target_side_length_m,
        grasp_offset_m,
        None if view is None else view.title,
    )


def render_capture(
    trace_path: Path,
    metrics_path: Path,
    output_dir: Path,
    width: int,
    height: int,
    fps: int,
    crf: int,
    gif_fps: int,
    gif_width: int,
) -> dict[str, object]:
    trace_path = trace_path.resolve()
    metrics_path = metrics_path.resolve()
    output_dir = output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    trace = _load_trace(trace_path)
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    target_center = np.asarray(metrics["target"]["initial_center_position_world_m"], dtype=np.float64)
    target_rotation = np.asarray(metrics["target"]["initial_rotation_world"], dtype=np.float64)
    grasp_offset_m = float(
        np.linalg.norm(metrics["target"]["grasp_offset_body_m"])
    )
    target_side_length_m = float(
        metrics["target"].get("side_length_m", 2.0 * grasp_offset_m)
    )
    terminal_shape_rad = float(metrics["candidate"]["terminal_arm_angle_rad"])
    sample = np.unique(np.linspace(0, len(trace["time"]) - 1, 120).astype(int))
    path_points = (
        trace["target_grasp_position"][sample]
        if "target_grasp_position" in trace
        else trace["desired_position"][sample]
    )

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
    indices = _video_indices(trace["time"], fps)

    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("ffmpeg is required")
    video_path = output_dir / "selected_capture_isometric.mp4"
    command = [
        ffmpeg, "-y", "-loglevel", "error", "-f", "rawvideo", "-vcodec", "rawvideo",
        "-pix_fmt", "rgb24", "-s", f"{width}x{height}", "-r", str(fps), "-i", "-",
        "-an", "-c:v", "libx264", "-preset", "fast", "-crf", str(crf),
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(video_path),
    ]
    flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.PIPE, creationflags=flags)
    assert process.stdin is not None
    storyboard_frames: list[Image.Image] = []
    storyboard_targets = {0, len(indices) // 2, len(indices) - 1}
    try:
        for output_index, trace_index in enumerate(indices):
            frame = _render_frame(
                model,
                data,
                renderer,
                trace,
                int(trace_index),
                target_center,
                target_rotation,
                target_side_length_m,
                grasp_offset_m,
                target_mocap_id,
                path_points,
                terminal_shape_rad,
            )
            process.stdin.write(np.ascontiguousarray(frame).tobytes())
            if output_index in storyboard_targets:
                storyboard_frames.append(Image.fromarray(frame).resize((480, 360), Image.Resampling.LANCZOS))
    finally:
        renderer.close()
        process.stdin.close()
    assert process.stderr is not None
    error = process.stderr.read().decode("utf-8", errors="replace")
    if process.wait() != 0:
        raise RuntimeError(error)

    gif_path = output_dir / "selected_capture_isometric.gif"
    gif_filter = (
        f"[0:v]fps={gif_fps},scale={gif_width}:-2:flags=lanczos,split[s0][s1];"
        "[s0]palettegen=max_colors=128:stats_mode=diff[p];"
        "[s1][p]paletteuse=dither=sierra2_4a:diff_mode=rectangle"
    )
    subprocess.run(
        [
            ffmpeg,
            "-y",
            "-loglevel",
            "error",
            "-i",
            str(video_path),
            "-filter_complex",
            gif_filter,
            "-loop",
            "0",
            str(gif_path),
        ],
        check=True,
        creationflags=flags,
    )

    storyboard_path = output_dir / "selected_capture_storyboard.png"
    canvas = Image.new("RGB", (1440, 360), (8, 13, 23))
    for position, frame in enumerate(storyboard_frames):
        canvas.paste(frame, (position * 480, 0))
    canvas.save(storyboard_path)

    ffprobe = shutil.which("ffprobe")
    if ffprobe is None:
        raise RuntimeError("ffprobe is required")
    probe = subprocess.run(
        [
            ffprobe, "-v", "error", "-count_frames", "-select_streams", "v:0",
            "-show_entries", "stream=width,height,avg_frame_rate,nb_read_frames:format=duration",
            "-of", "json", str(video_path),
        ],
        check=True,
        capture_output=True,
        text=True,
        creationflags=flags,
    )
    payload = json.loads(probe.stdout)
    stream = payload["streams"][0]
    with Image.open(gif_path) as gif_image:
        gif_frames = int(getattr(gif_image, "n_frames", 1))
        gif_size = tuple(int(value) for value in gif_image.size)
        gif_duration_s = sum(
            float(gif_image.seek(frame_index) or gif_image.info.get("duration", 0.0))
            for frame_index in range(gif_frames)
        ) / 1000.0
    report: dict[str, object] = {
        "material_passport": {
            "origin_skill": "visualize",
            "origin_mode": "verified-trace-render",
            "verification_status": "VERIFIED",
            "version_label": "fpmfc_capture_visualization_v1",
        },
        "source_trace": str(trace_path.relative_to(PROJECT_ROOT)),
        "source_trace_sha256": _sha256(trace_path),
        "source_metrics": str(metrics_path.relative_to(PROJECT_ROOT)),
        "source_metrics_sha256": _sha256(metrics_path),
        "video": {
            "path": str(video_path.relative_to(PROJECT_ROOT)),
            "sha256": _sha256(video_path),
            "bytes": video_path.stat().st_size,
            "width": int(stream["width"]),
            "height": int(stream["height"]),
            "frames": int(stream["nb_read_frames"]),
            "duration_s": float(payload["format"]["duration"]),
            "fps": fps,
        },
        "storyboard": {
            "path": str(storyboard_path.relative_to(PROJECT_ROOT)),
            "sha256": _sha256(storyboard_path),
            "bytes": storyboard_path.stat().st_size,
            "width": 1440,
            "height": 360,
        },
        "gif": {
            "path": str(gif_path.relative_to(PROJECT_ROOT)),
            "sha256": _sha256(gif_path),
            "bytes": gif_path.stat().st_size,
            "width": gif_size[0],
            "height": gif_size[1],
            "frames": gif_frames,
            "duration_s": gif_duration_s,
            "fps": gif_fps,
        },
        "checks": {
            "source_trace_matches_metrics": metrics["trace"]["sha256"] == _sha256(trace_path),
            "video_dimensions_match": int(stream["width"]) == width and int(stream["height"]) == height,
            "video_frame_count_matches": int(stream["nb_read_frames"]) == len(indices),
            "video_duration_matches": abs(float(payload["format"]["duration"]) - len(indices) / fps) <= 1.0 / fps,
            "storyboard_complete": len(storyboard_frames) == 3 and storyboard_path.stat().st_size > 20_000,
            "gif_dimensions_match": gif_size[0] == gif_width,
            "gif_frame_count_matches": abs(gif_frames - round(float(trace["time"][-1]) * gif_fps) - 1) <= 2,
            "gif_duration_matches": abs(gif_duration_s - float(trace["time"][-1])) <= 2.0 / gif_fps,
        },
    }
    report["passed"] = all(report["checks"].values())
    manifest_path = output_dir / "visualization_manifest.json"
    manifest_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    if not report["passed"]:
        raise RuntimeError("capture visualization validation failed")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--metrics", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--crf", type=int, default=20)
    parser.add_argument("--gif-fps", type=int, default=12)
    parser.add_argument("--gif-width", type=int, default=720)
    args = parser.parse_args()
    if args.gif_fps <= 0 or args.gif_width <= 0:
        raise ValueError("gif fps and width must be positive")
    report = render_capture(
        args.trace,
        args.metrics,
        args.output_dir,
        args.width,
        args.height,
        args.fps,
        args.crf,
        args.gif_fps,
        args.gif_width,
    )
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
