"""Render the three authoritative contact traces without rerunning dynamics."""

import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from plot_style import OUT, ROOT

sys.path.insert(0, str(ROOT))
from v6_mujoco.fpmfc.contact_model import default_contact_model_spec

SOURCE = ROOT / "output" / "fpmfc" / "contact"
RUNS = [
    ("n102_rigid_authoritative", "N102 rigid position"),
    ("n103_admittance_authoritative", "N103 admittance + shape"),
    ("n104_admittance_no_shape_authoritative", "N104 admittance - shape"),
]
WIDTH, HEIGHT, FPS = 640, 480, 15


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def annotate(rgb, label, time_s, force_n, contact, passed):
    canvas = Image.fromarray(rgb).convert("RGB")
    draw = ImageDraw.Draw(canvas)
    font = ImageFont.truetype("C:/Windows/Fonts/arial.ttf", 17)
    small = ImageFont.truetype("C:/Windows/Fonts/arial.ttf", 14)
    draw.rectangle((0, 0, WIDTH, 73), fill=(10, 16, 25))
    draw.text((14, 7), label, font=font, fill=(245, 248, 250))
    draw.text((14, 34), f"t={time_s:.2f} s    normal force={force_n:.2f} N"
              f"    contact={'yes' if contact else 'no'}", font=small,
              fill=(210, 223, 235))
    draw.rectangle((0, HEIGHT - 33, WIDTH, HEIGHT), fill=(10, 16, 25))
    verdict = "all gates passed" if passed else "force-tracking gate failed"
    draw.text((14, HEIGHT - 26), verdict, font=small,
              fill=(111, 214, 164) if passed else (255, 180, 100))
    return np.asarray(canvas)


def render_one(model, source_name, label):
    source = SOURCE / source_name
    trace_path = source / "trace.npz"
    metrics_path = source / "metrics.json"
    validation_path = source / "validation.json"
    original = json.loads((source / "artifact_manifest.json").read_text(encoding="utf-8"))
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    validation = json.loads(validation_path.read_text(encoding="utf-8"))
    for filename in ("trace.npz", "metrics.json", "validation.json"):
        if sha256(source / filename) != original["artifacts"][filename]:
            raise RuntimeError(f"source artifact hash mismatch: {source / filename}")
    if not validation["passed"]:
        raise RuntimeError(f"independent replay failed: {source_name}")
    with np.load(trace_path, allow_pickle=False) as archive:
        time = archive["time"]
        qpos = archive["qpos"]
        qvel = archive["qvel"]
        force = archive["measured_normal_force_n"]
        contact = archive["contact_detected"]
    samples = np.clip(np.searchsorted(time, np.arange(FPS + 1) / FPS), 0, len(time) - 1)
    data = mujoco.MjData(model)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.azimuth = 136
    camera.elevation = -21
    camera.distance = 1.65
    renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)
    flange_site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "flange_site")
    grasp_site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "target_grasp_site")
    destination = OUT / f"contact_{source_name.split('_')[0]}"
    destination.mkdir(parents=True, exist_ok=True)
    movie = destination / "contact_motion.mp4"
    command = [
        shutil.which("ffmpeg"), "-y", "-loglevel", "error", "-f", "rawvideo",
        "-pixel_format", "rgb24", "-video_size", f"{WIDTH}x{HEIGHT}",
        "-framerate", str(FPS), "-i", "-", "-an", "-c:v", "libx264",
        "-preset", "fast", "-crf", "20", "-pix_fmt", "yuv420p",
        "-movflags", "+faststart", str(movie),
    ]
    flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.PIPE,
                               creationflags=flags)
    storyboard = []
    try:
        for frame_number, sample in enumerate(samples):
            data.qpos[:] = qpos[sample]
            data.qvel[:] = qvel[sample]
            data.time = float(time[sample])
            mujoco.mj_forward(model, data)
            camera.lookat[:] = 0.5 * (data.site_xpos[flange_site] + data.site_xpos[grasp_site])
            renderer.update_scene(data, camera=camera)
            frame = annotate(renderer.render(), label, float(time[sample]),
                             float(force[sample]), bool(contact[sample]),
                             bool(metrics["acceptance"]["passed"]))
            process.stdin.write(np.ascontiguousarray(frame).tobytes())
            if frame_number in (0, FPS // 2, FPS):
                storyboard.append(Image.fromarray(frame).resize((480, 360)))
    finally:
        renderer.close()
        if process.stdin:
            process.stdin.close()
    stderr = process.stderr.read().decode("utf-8", errors="replace")
    if process.wait() != 0:
        raise RuntimeError(stderr)
    probe = subprocess.run(
        [shutil.which("ffprobe"), "-v", "error", "-count_frames", "-select_streams", "v:0",
         "-show_entries", "stream=width,height,nb_read_frames", "-of", "json", str(movie)],
        check=True, capture_output=True, text=True, creationflags=flags,
    )
    stream = json.loads(probe.stdout)["streams"][0]
    if (int(stream["width"]), int(stream["height"]), int(stream["nb_read_frames"])) != (
        WIDTH, HEIGHT, len(samples)
    ):
        raise RuntimeError(f"video frame contract failed: {source_name}")
    sheet = Image.new("RGB", (1440, 360))
    for index, frame in enumerate(storyboard):
        sheet.paste(frame, (index * 480, 0))
    sheet_path = destination / "contact_storyboard.png"
    sheet.save(sheet_path)
    report = {
        "source": str(trace_path.relative_to(ROOT)),
        "source_sha256": sha256(trace_path),
        "source_manifest_verified": True,
        "independent_replay_verified": True,
        "model_runtime_contract_sha256": metrics["model_identity"]["runtime_contract_sha256"],
        "video": str(movie.relative_to(ROOT)),
        "video_sha256": sha256(movie),
        "frames": len(samples),
        "resolution": [WIDTH, HEIGHT],
        "video_frame_contract_verified": True,
        "storyboard": str(sheet_path.relative_to(ROOT)),
        "storyboard_sha256": sha256(sheet_path),
        "acceptance_passed": bool(metrics["acceptance"]["passed"]),
    }
    (destination / "visualization_manifest.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )
    print(f"Rendered {label}: {movie}")


def main():
    spec = default_contact_model_spec()
    identity = spec.identity()["runtime_contract_sha256"]
    model = spec.compile_model()
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, WIDTH)
    model.vis.global_.offheight = max(model.vis.global_.offheight, HEIGHT)
    for source_name, label in RUNS:
        metrics = json.loads((SOURCE / source_name / "metrics.json").read_text(encoding="utf-8"))
        if metrics["model_identity"]["runtime_contract_sha256"] != identity:
            raise RuntimeError(f"render model differs from source: {source_name}")
        render_one(model, source_name, label)


if __name__ == "__main__":
    main()
