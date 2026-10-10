"""Run from any directory in this checked-out repository."""
import runpy
from pathlib import Path
p=Path(__file__).resolve()
root=next(x for x in p.parents if (x/"v6_mujoco").is_dir())
mod=runpy.run_path(str(root/"tools/s03_scientific_figures.py"))
mod["main"]("forecast_errors")
