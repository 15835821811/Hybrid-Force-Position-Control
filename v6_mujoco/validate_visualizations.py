"""Independently verify visualization hashes, images, and full video decoding."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
from pathlib import Path
from typing import Any

from PIL import Image

from .model import PROJECT_ROOT


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _resolve(relative: str) -> Path:
    path = (PROJECT_ROOT / relative).resolve()
    path.relative_to(PROJECT_ROOT.resolve())
    return path


def _decode_video(path: Path) -> bool:
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("ffmpeg is required for visualization validation")
    flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    result = subprocess.run(
        [ffmpeg, "-v", "error", "-i", str(path), "-map", "0:v:0", "-f", "null", "-"],
        capture_output=True,
        creationflags=flags,
    )
    return result.returncode == 0 and not result.stderr


def validate(output_dir: Path) -> dict[str, Any]:
    output_dir = Path(output_dir).resolve()
    manifest_path = output_dir / "visualization_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    figure_checks = []
    for item in manifest["figures"] + [manifest["preview"]]:
        path = _resolve(item["path"])
        try:
            with Image.open(path) as image:
                image.verify()
            with Image.open(path) as image:
                dimensions = image.size
            valid = dimensions == (item["width"], item["height"]) and _sha256(path) == item["sha256"]
        except Exception:
            valid = False
        figure_checks.append({"path": item["path"], "valid": valid})

    video_checks = []
    for item in manifest["videos"]:
        path = _resolve(item["path"])
        valid = path.is_file() and _sha256(path) == item["sha256"] and _decode_video(path)
        video_checks.append({"path": item["path"], "valid": valid, "frames": item["frames"], "duration_s": item["duration_s"]})

    source_metrics = _resolve(manifest["source_metrics"])
    source_trace = _resolve(manifest["source_trace"])
    checks = {
        "generator_report_passed": manifest["passed"] is True,
        "all_six_png_files_decode_and_match_hashes": len(figure_checks) == 6 and all(item["valid"] for item in figure_checks),
        "five_complete_mp4_streams_decode_and_match_hashes": len(video_checks) == 5 and all(item["valid"] for item in video_checks),
        "source_metrics_hash_matches": _sha256(source_metrics) == manifest["source_metrics_sha256"],
        "source_trace_hash_matches": _sha256(source_trace) == manifest["source_trace_sha256"],
        "five_camera_definitions_are_unique": len({item["key"] for item in manifest["views"]}) == 5,
        "coordinate_frame_contract_matches_v6_style": (
            manifest["coordinate_frames"]["axis_color_order"] == ["x_red", "y_green", "z_blue"]
            and manifest["coordinate_frames"]["target_axis_length_m"] > manifest["coordinate_frames"]["end_effector_axis_length_m"]
            and set(manifest["coordinate_frames"]["video_frames_rendered"]) == {"live_target_point_frame", "live_flexiv_flange_frame", "W1_to_W7_target_frames"}
            and set(manifest["coordinate_frames"]["path_figure_frames_rendered"]) == {"W1_to_W7_target_frames", "final_target_frame", "final_flexiv_flange_frame"}
        ),
        "video_contains_full_target_path_and_seven_waypoints": (
            manifest["video_target_path"]["geometry"] == "W1_to_W7_capsule_polyline"
            and manifest["video_target_path"]["waypoint_count"] == 7
            and manifest["video_target_path"]["waypoint_markers"] == "gold_spheres"
        ),
    }
    report = {
        "passed": all(checks.values()),
        "checks": checks,
        "figures": figure_checks,
        "videos": video_checks,
        "manifest": str(manifest_path.relative_to(PROJECT_ROOT)),
        "manifest_sha256": _sha256(manifest_path),
    }
    report_path = output_dir / "visualization_validation.json"
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=PROJECT_ROOT / "output" / "visualization")
    args = parser.parse_args()
    report = validate(args.output_dir)
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
