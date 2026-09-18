"""Generate a vector PSO convergence figure from optimizer JSON files."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from paper_plot_style import COLORS, plt, save_figure  # noqa: E402


def generate(inputs: list[Path], labels: list[str], output: Path) -> Path:
    if len(inputs) != len(labels):
        raise ValueError("one label is required per optimizer JSON")
    figure, axis = plt.subplots(figsize=(3.45, 2.35))
    all_values: list[float] = []
    for index, (path, label) in enumerate(zip(inputs, labels)):
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        values = np.asarray(payload["optimizer"]["history"], dtype=np.float64)
        if values.ndim != 1 or not len(values) or np.any(~np.isfinite(values)):
            raise ValueError(f"invalid PSO history in {path}")
        generations = np.arange(1, len(values) + 1)
        axis.step(
            generations,
            values,
            where="post",
            color=COLORS[index % len(COLORS)],
            label=label,
        )
        all_values.extend(values.tolist())
    positive = [value for value in all_values if value > 0.0]
    if positive and max(positive) / min(positive) >= 100.0:
        axis.set_yscale("log")
    axis.set_xlabel("PSO generation")
    axis.set_ylabel("Best-so-far objective")
    axis.legend(frameon=False)
    figure.tight_layout(pad=0.3)
    return save_figure(figure, output)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inputs", nargs="+", type=Path, required=True)
    parser.add_argument("--labels", nargs="+", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    generate(args.inputs, args.labels, args.output)


if __name__ == "__main__":
    main()
