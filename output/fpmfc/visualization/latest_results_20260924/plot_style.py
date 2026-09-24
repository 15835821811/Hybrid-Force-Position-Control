"""Shared vector/raster style for the current FPMFC result atlas."""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]

plt.rcParams.update(
    {
        "font.family": "DejaVu Sans",
        "font.size": 9,
        "axes.labelsize": 9,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "legend.fontsize": 8,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "savefig.dpi": 240,
        "figure.dpi": 120,
    }
)

COLORS = {
    "full": "#0072B2",
    "no-shape": "#D55E00",
    "no-base-reaction": "#009E73",
    "fixed0": "#D55E00",
    "fixedpi2": "#CC79A7",
    "rigid": "#4D4D4D",
    "admittance": "#0072B2",
    "admittance-no-shape": "#D55E00",
}


def finish(fig, stem):
    fig.tight_layout(pad=1.4)
    for ext in ("pdf", "png"):
        fig.savefig(OUT / f"{stem}.{ext}", bbox_inches="tight", pad_inches=0.07)
    plt.close(fig)
    print(f"Saved {stem}.pdf and .png")
