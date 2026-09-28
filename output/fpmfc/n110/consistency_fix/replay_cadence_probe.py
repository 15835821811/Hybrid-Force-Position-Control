"""Diagnostic only: compare torque replay with original forward-call cadence."""

import json
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))

from v6_mujoco.model import PROJECT_ROOT, geom_id
from v6_mujoco.fpmfc.contact_model import default_contact_model_spec


def run(trace_path: Path, mode: str):
    spec = default_contact_model_spec()
    model = spec.compile_model()
    model.geom_pos[geom_id(model, "workspace_obstacle_0")] = [10.0, 10.0, 10.0]
    data = mujoco.MjData(model)
    with np.load(trace_path, allow_pickle=False) as trace:
        for name, key in (("qpos", "initial_qpos"), ("qvel", "initial_qvel"),
                          ("ctrl", "initial_ctrl"), ("qfrc_applied", "initial_qfrc_applied"),
                          ("xfrc_applied", "initial_xfrc_applied"),
                          ("qacc_warmstart", "initial_qacc_warmstart"),
                          ("mocap_pos", "initial_mocap_pos"), ("mocap_quat", "initial_mocap_quat"),
                          ("eq_active", "initial_eq_active"), ("act", "initial_act"),
                          ("plugin_state", "initial_plugin_state")):
            getattr(data, name)[:] = trace[key]
        max_qpos = max_qvel = 0.0
        errors = []
        for i, torque in enumerate(trace["torque"]):
            if mode == "servo" or mode == "task_servo":
                if mode == "task_servo" and i % 10 == 0:
                    mujoco.mj_forward(model, data)
                mujoco.mj_forward(model, data)
            data.ctrl[:] = torque
            mujoco.mj_step(model, data)
            qpos = float(np.max(np.abs(data.qpos-trace["qpos"][i])))
            qvel = float(np.max(np.abs(data.qvel-trace["qvel"][i])))
            max_qpos = max(max_qpos, qpos)
            max_qvel = max(max_qvel, qvel)
            if i in (0, 9, 10, 100, 200, 300, 400, 499):
                errors.append({"step": i, "qpos": qpos, "qvel": qvel})
        return {"mode": mode, "qpos_max": max_qpos, "qvel_max": max_qvel, "selected_errors": errors}


if __name__ == "__main__":
    path = PROJECT_ROOT / "output/fpmfc/n110/consistency_fix/A/admittance-no-shape/trace.npz"
    result = [run(path, mode) for mode in ("direct", "servo", "task_servo")]
    print(json.dumps(result, indent=2))
