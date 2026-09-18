"""Interactive replay of a generated MuJoCo torque trace."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from .model import PROJECT_ROOT, default_model_spec, geom_id


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", nargs="?", type=Path, default=PROJECT_ROOT / "output" / "traces" / "flexiv_mujoco_scenario_00.npz")
    parser.add_argument("--speed", type=float, default=1.0)
    args = parser.parse_args()
    trace = np.load(args.trace, allow_pickle=False)
    spec = default_model_spec()
    model = spec.compile_model()
    model.geom_pos[geom_id(model, "workspace_obstacle_0")] = trace["obstacle_position"]
    model.geom_contype[:] = 0
    model.geom_conaffinity[:] = 0
    data = mujoco.MjData(model)
    data.qpos[:] = trace["initial_qpos"]
    data.qvel[:] = trace["initial_qvel"]
    mujoco.mj_forward(model, data)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        for index, torque in enumerate(trace["torque"]):
            if not viewer.is_running():
                break
            started = time.perf_counter()
            data.ctrl[:] = torque
            data.mocap_pos[0] = trace["target_position"][index]
            mujoco.mj_step(model, data)
            viewer.sync()
            delay = spec.timestep_s / max(args.speed, 1e-6) - (time.perf_counter() - started)
            if delay > 0.0:
                time.sleep(delay)


if __name__ == "__main__":
    main()
