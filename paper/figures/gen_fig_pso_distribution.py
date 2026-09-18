"""Plot across-seed feasible objectives and feasibility rates."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from paper_plot_style import COLORS, plt, save_figure  # noqa: E402


DISPLAY_NAMES = {
    "joint": "Joint $T,\\psi$",
    "fixed0": "$\\psi=0$",
    "fixedpi2": "$\\psi=\\pi/2$",
}


def generate(summary: Path, output: Path) -> Path:
    payload = json.loads(Path(summary).read_text(encoding="utf-8"))
    variants = payload["variants"]
    names = list(variants)
    figure, axes = plt.subplots(1, 2, figsize=(7.05, 2.35))
    feasible_objectives: list[list[float]] = []
    for name in names:
        values = [
            float(run["best_objective"])
            for run in variants[name]["runs"]
            if bool(run["feasible"])
        ]
        feasible_objectives.append(values)
    positions = np.arange(1, len(names) + 1)
    all_feasible = [value for values in feasible_objectives for value in values]
    if all_feasible:
        value_min = min(all_feasible)
        value_max = max(all_feasible)
        value_span = max(value_max - value_min, 0.08 * max(abs(value_max), 1.0))
        lower_limit = value_min - 0.18 * value_span
        upper_limit = value_max + 0.22 * value_span
    else:
        lower_limit, upper_limit = 0.0, 1.0
    for index, (position, values) in enumerate(zip(positions, feasible_objectives)):
        color = COLORS[index % len(COLORS)]
        if values:
            box = axes[0].boxplot(
                [values],
                positions=[position],
                widths=0.52,
                patch_artist=True,
                showfliers=False,
            )
            box["boxes"][0].set_facecolor(color)
            box["boxes"][0].set_alpha(0.35)
            jitter = np.linspace(-0.10, 0.10, len(values)) if len(values) > 1 else [0.0]
            axes[0].scatter(
                position + np.asarray(jitter),
                values,
                s=14,
                color=color,
                zorder=3,
            )
        else:
            marker_y = lower_limit + 0.06 * (upper_limit - lower_limit)
            axes[0].scatter(
                [position], [marker_y], marker="x", s=30, color="#B2182B", zorder=4
            )
            axes[0].text(
                position,
                marker_y + 0.04 * (upper_limit - lower_limit),
                "none",
                ha="center",
                va="bottom",
                fontsize=7,
                color="#B2182B",
            )
    axes[0].set_xticks(positions, [DISPLAY_NAMES.get(name, name) for name in names])
    axes[0].set_ylabel("Best feasible objective")
    axes[0].set_ylim(lower_limit, upper_limit)
    rates = [float(variants[name]["feasible_seed_fraction"]) for name in names]
    bars = axes[1].bar(
        positions,
        rates,
        width=0.62,
        color=[COLORS[index % len(COLORS)] for index in range(len(names))],
    )
    axes[1].set_xticks(positions, [DISPLAY_NAMES.get(name, name) for name in names])
    axes[1].set_ylabel("Feasible-seed fraction")
    axes[1].set_ylim(0.0, 1.08)
    for bar, value in zip(bars, rates):
        axes[1].text(
            bar.get_x() + bar.get_width() / 2.0,
            value + 0.025,
            f"{value:.0%}",
            ha="center",
            va="bottom",
            fontsize=8,
        )
    figure.tight_layout(w_pad=1.2, pad=0.25)
    return save_figure(figure, output)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    generate(args.summary, args.output)


if __name__ == "__main__":
    main()
