"""Generate paper-scale pre-contact time histories from saved NPZ traces."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from paper_plot_style import COLORS, plt, save_figure  # noqa: E402


def _parse_trace(value: str) -> tuple[str, Path]:
    if "=" not in value:
        raise argparse.ArgumentTypeError("trace must use LABEL=PATH")
    label, path = value.split("=", 1)
    if not label.strip():
        raise argparse.ArgumentTypeError("trace label cannot be empty")
    return label.strip(), Path(path)


def generate(traces: list[tuple[str, Path]], output: Path) -> Path:
    figure, axes = plt.subplots(1, 3, figsize=(7.05, 2.15))
    for index, (label, path) in enumerate(traces):
        with np.load(path, allow_pickle=False) as archive:
            time_s = np.asarray(archive["time"], dtype=np.float64)
            base_twist = np.asarray(archive["base_twist"], dtype=np.float64)
            position_error_mm = 1000.0 * np.asarray(
                archive["position_error_m"], dtype=np.float64
            )
            arm_angle_deg = np.rad2deg(
                np.unwrap(np.asarray(archive["arm_angle_rad"], dtype=np.float64))
            )
            desired_arm_angle_deg = np.rad2deg(
                np.unwrap(
                    np.asarray(archive["desired_arm_angle_rad"], dtype=np.float64)
                )
            )
        color = COLORS[index % len(COLORS)]
        axes[0].plot(time_s, np.linalg.norm(base_twist[:, 3:], axis=1), color=color, label=label)
        axes[1].plot(time_s, position_error_mm, color=color, label=label)
        axes[2].plot(time_s, arm_angle_deg, color=color, label=f"{label}: actual")
        axes[2].plot(
            time_s,
            desired_arm_angle_deg,
            color=color,
            linestyle="--",
            linewidth=1.0,
            label=f"{label}: desired",
        )
    axes[0].set_xlabel("Time (s)")
    axes[0].set_ylabel(r"Base $\|\omega_b\|$ (rad/s)")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Position error (mm)")
    axes[2].set_xlabel("Time (s)")
    axes[2].set_ylabel(r"Arm-shape angle $\psi$ (deg)")
    axes[0].legend(frameon=False)
    axes[2].legend(frameon=False, ncol=1)
    figure.tight_layout(w_pad=1.0, pad=0.25)
    return save_figure(figure, output)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--traces", nargs="+", type=_parse_trace, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    generate(args.traces, args.output)


if __name__ == "__main__":
    main()
