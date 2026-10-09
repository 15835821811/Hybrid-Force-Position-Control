from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from v6_mujoco.model import PROJECT_ROOT
from v6_mujoco.postgrasp.physics import (constraint_jacobian, equality_rows,
                                        mass_matrix, object_id)

ROOT = PROJECT_ROOT / "output/fpmfc/n201_interface_calibration"
OLD = PROJECT_ROOT / "output/fpmfc/n200_postgrasp"
MODEL = PROJECT_ROOT / "models/flexiv_rizon4s_postgrasp_scene.xml"
NAME = "postgrasp_latch"


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def save(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False)+"\n", encoding="utf-8")


def relative_rotation_error(RA: np.ndarray, RB: np.ndarray, frozen: np.ndarray) -> float:
    return float(Rotation.from_matrix(frozen.T @ RA.T @ RB).magnitude())


def frames(model, data):
    a = object_id(model, mujoco.mjtObj.mjOBJ_SITE, "postgrasp_tool_interface")
    b = object_id(model, mujoco.mjtObj.mjOBJ_SITE, "target_grasp_site")
    RA = np.asarray(data.site_xmat[a]).reshape(3,3)
    RB = np.asarray(data.site_xmat[b]).reshape(3,3)
    return RA, RB, np.asarray(data.site_xpos[a]), np.asarray(data.site_xpos[b])


def pose_reference(model, data):
    RA, RB, a, b = frames(model,data)
    return {"rotation": (RA.T@RB).copy(), "translation": (RA.T@(b-a)).copy()}


def pose(model, data, reference):
    RA, RB, a, b = frames(model,data)
    vector = RA.T@(b-a)-reference["translation"]
    return {"translation_error_m": float(np.linalg.norm(vector)),
            "translation_error_tool_m": vector.tolist(),
            "rotation_error_deg": float(np.rad2deg(relative_rotation_error(RA,RB,reference["rotation"]))),
            "relative_rotation": (RA.T@RB).tolist()}


def solver_diagnostics(model, data):
    rows = equality_rows(model,data,NAME)
    J = constraint_jacobian(model,data)
    force = np.asarray(data.efc_force)
    correction = np.linalg.solve(mass_matrix(model,data),J.T@force)
    kkt = np.asarray(data.efc_b)[rows]+J[rows]@correction+np.asarray(data.efc_R)[rows]*force[rows]
    return {"equality_kkt_acceleration_residual": kkt.tolist(),
            "equality_kkt_max_absolute": float(np.max(np.abs(kkt))),
            "solver_niter": np.asarray(data.solver_niter).tolist(),
            "efc_pos": np.asarray(data.efc_pos)[rows].tolist(),
            "efc_velocity": (J[rows]@data.qvel).tolist(),
            "efc_KBIP": np.asarray(data.efc_KBIP)[rows].tolist()}


def runtime_contract(model, data):
    eid = object_id(model,mujoco.mjtObj.mjOBJ_EQUALITY,NAME)
    kind = int(model.eq_objtype[eid])
    obj1, obj2 = int(model.eq_obj1id[eid]), int(model.eq_obj2id[eid])
    enum_kind = mujoco.mjtObj(kind)
    refsafe = not bool(model.opt.disableflags & int(mujoco.mjtDisableBit.mjDSBL_REFSAFE))
    raw = float(model.eq_solref[eid,0])
    return {"definition": "body_based" if kind == int(mujoco.mjtObj.mjOBJ_BODY) else "site_based",
            "equality_type":int(model.eq_type[eid]),"object_type":kind,
            "object1":mujoco.mj_id2name(model,enum_kind,obj1),
            "object2":mujoco.mj_id2name(model,enum_kind,obj2),
            "active":bool(data.eq_active[eid]),"eq_data_raw":model.eq_data[eid].tolist(),
            "solref_raw":model.eq_solref[eid].tolist(),"solimp":model.eq_solimp[eid].tolist(),
            "torquescale_m":float(model.eq_data[eid,10]),"refsafe":refsafe,
            "timeconst_effective_s":max(raw,2*model.opt.timestep) if raw>0 and refsafe else raw,
            "timestep_s":float(model.opt.timestep),"integrator":int(model.opt.integrator),
            "solver":int(model.opt.solver),"iterations":int(model.opt.iterations),
            "tolerance":float(model.opt.tolerance),"disableflags":int(model.opt.disableflags),
            "enableflags":int(model.opt.enableflags)}
