"""Shared publication style for data-driven FPMFC figures."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


FONT_SIZE = 9
COLORS = (
    "#0072B2",  # blue
    "#D55E00",  # vermillion
    "#009E73",  # green
    "#CC79A7",  # purple
    "#E69F00",  # orange
    "#56B4E9",  # sky blue
)

matplotlib.rcParams.update(
    {
        "font.size": FONT_SIZE,
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
        "axes.labelsize": FONT_SIZE,
        "axes.titlesize": FONT_SIZE,
        "xtick.labelsize": FONT_SIZE - 1,
        "ytick.labelsize": FONT_SIZE - 1,
        "legend.fontsize": FONT_SIZE - 1,
        "figure.dpi": 150,
        "savefig.dpi": 300,
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.04,
        "axes.grid": True,
        "grid.alpha": 0.20,
        "grid.linewidth": 0.5,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "lines.linewidth": 1.4,
        "mathtext.fontset": "stix",
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
    }
)


def save_figure(figure: plt.Figure, output: Path) -> Path:
    path = Path(output).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, format="pdf")
    plt.close(figure)
    if not path.is_file() or path.stat().st_size == 0:
        raise RuntimeError(f"figure was not written: {path}")
    print(f"saved={path}")
    return path
