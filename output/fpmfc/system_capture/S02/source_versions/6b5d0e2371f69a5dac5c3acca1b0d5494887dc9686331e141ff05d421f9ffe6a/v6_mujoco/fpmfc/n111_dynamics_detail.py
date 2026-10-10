"""Read-only split of robot interface load and target reaction constraints."""

from __future__ import annotations

import json

import mujoco
import numpy as np

from ..model import geom_id, site_id
from .contact_config import load_contact_config
from .contact_model import default_contact_model_spec
from .contact_observation import ContactObserver
from .n111_contract import CELLS, OUTPUT_ROOT


def analyze() -> dict:
    spec = default_contact_model_spec()
    model = spec.compile_model()
    model.geom_pos[geom_id(model, "workspace_obstacle_0")] = [10.0, 10.0, 10.0]
    cfg = load_contact_config(spec.contact_config_path)
    observer = ContactObserver(model, tool_face_recess_m=float(cfg["interface"]["tool_pad_face_recess_m"]))
    base = np.arange(spec.base_slices(model)[1].start, spec.base_slices(model)[1].stop)
    joints = spec.joint_addresses(model)[1]
    target = np.arange(spec.target_slices(model)[1].start, spec.target_slices(model)[1].stop)
    robot = np.r_[base, joints]
    summary = {}
    for cell in CELLS:
        path = OUTPUT_ROOT / cell
        values = {key: [] for key in ("robot_constraint_minus_interface_norm", "target_constraint_reaction_norm",
                                          "interface_robot_norm", "selected_interface_contact_count",
                                          "other_contact_count", "joint_limit_count")}
        with np.load(path / "trace.npz", allow_pickle=False) as t:
            for i in range(500):
                data = mujoco.MjData(model)
                data.time = float(t["feedback_time_s"][i])
                data.qpos[:] = t["feedback_qpos"][i]
                data.qvel[:] = t["feedback_qvel"][i]
                data.ctrl[:] = t["feedback_ctrl_used_for_forward"][i]
                data.qacc_warmstart[:] = t["initial_qacc_warmstart"] if i == 0 else t["qacc_warmstart_used_for_forward"][i-1]
                data.mocap_pos[:] = t["initial_mocap_pos"] if i == 0 else t["mocap_pos"][i-1]
                data.mocap_quat[:] = t["initial_mocap_quat"] if i == 0 else t["mocap_quat"][i-1]
                o = observer.observe(data)
                jp, jr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
                mujoco.mj_jacSite(model, o.forward_data, jp, jr, site_id(model, "flange_site"))
                f = jp.T @ o.contact_force_world_n + jr.T @ o.contact_torque_at_flange_world_nm
                constraint = np.asarray(o.forward_data.qfrc_constraint)
                values["robot_constraint_minus_interface_norm"].append(float(np.linalg.norm(constraint[robot]-f[robot])))
                values["target_constraint_reaction_norm"].append(float(np.linalg.norm(constraint[target])))
                values["interface_robot_norm"].append(float(np.linalg.norm(f[robot])))
                values["selected_interface_contact_count"].append(o.contact_count)
                values["other_contact_count"].append(sum(
                    {int(o.forward_data.contact[k].geom1), int(o.forward_data.contact[k].geom2)} != {observer.pad, observer.plate}
                    for k in range(o.forward_data.ncon)))
                values["joint_limit_count"].append(int(np.count_nonzero(
                    np.asarray(o.forward_data.efc_type[:o.forward_data.nefc]) == int(mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT))))
        arrays = {key: np.asarray(value) for key, value in values.items()}
        np.savez_compressed(path / "dynamics_detail.npz", **arrays)
        summary[cell] = {key: {"max": float(np.max(value)), "rms": float(np.sqrt(np.mean(value**2)))}
                         for key, value in arrays.items()}
    result = {"schema_version": "n111_dynamics_detail_v1", "cells": summary,
              "interpretation": "robot constraint minus interface wrench excludes target DOFs; target constraint is the interface counterpart, not an unmodeled robot contact"}
    (OUTPUT_ROOT / "dynamics_detail_summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    return result


if __name__ == "__main__":
    print(json.dumps({cell: row["robot_constraint_minus_interface_norm"] for cell, row in analyze()["cells"].items()}, indent=2))
