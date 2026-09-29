"""Physical observables and constraint-consistent initial state for N200."""

from __future__ import annotations

import hashlib
from pathlib import Path

import mujoco
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from v6_mujoco.model import PROJECT_ROOT, default_model_spec
from v6_mujoco.fpmfc.target import target_from_config


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def object_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    result = mujoco.mj_name2id(model, kind, name)
    if result < 0:
        raise ValueError(f"missing {kind}: {name}")
    return int(result)


def joint_slices(model: mujoco.MjModel, name: str) -> tuple[slice, slice]:
    jid = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    nq, nv = (7, 6) if model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_FREE else (1, 1)
    q = int(model.jnt_qposadr[jid]); v = int(model.jnt_dofadr[jid])
    return slice(q, q+nq), slice(v, v+nv)


def load_model(path: Path, timestep: float | None = None) -> mujoco.MjModel:
    source = default_model_spec()
    _, assets = source._xml_and_assets()
    xml = path.read_text(encoding="utf-8").replace('meshdir="../assets/meshes"', 'meshdir="."')
    model = mujoco.MjModel.from_xml_string(xml, assets)
    if (model.nq, model.nv, model.nu, model.neq) != (21, 19, 7, 1):
        raise RuntimeError("unexpected postgrasp model dimensions")
    if np.linalg.norm(model.opt.gravity) > 1e-12 or model.opt.integrator != mujoco.mjtIntegrator.mjINT_RK4:
        raise RuntimeError("postgrasp model must have zero gravity and RK4")
    original_xml = PROJECT_ROOT / "models/flexiv_rizon4s_contact_scene.xml"
    old_xml = original_xml.read_text(encoding="utf-8").replace('meshdir="../assets/meshes"', 'meshdir="."')
    original = mujoco.MjModel.from_xml_string(old_xml, assets)
    for name in ("tumbling_target", "base_link_0", "base_link", "link1", "link2", "link3", "link4", "link5", "link6", "link7", "flange"):
        i = object_id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        j = object_id(original, mujoco.mjtObj.mjOBJ_BODY, name)
        if not (np.array_equal(model.body_mass[i], original.body_mass[j]) and
                np.array_equal(model.body_inertia[i], original.body_inertia[j])):
            raise RuntimeError(f"mass or inertia changed for {name}")
    for name in ("base_free_joint", "target_free_joint"):
        jid = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if model.jnt_type[jid] != mujoco.mjtJoint.mjJNT_FREE:
            raise RuntimeError(f"{name} is not free")
    for name in ("gripper_contact_pad", "target_contact_plate"):
        gid = object_id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if model.geom_contype[gid] or model.geom_conaffinity[gid]:
            raise RuntimeError("duplicate latch interface contact remains enabled")
    if timestep is not None:
        model.opt.timestep = timestep
    return model


def transfer_c1(model: mujoco.MjModel, data: mujoco.MjData, source: Path, target_config: Path) -> dict:
    cfg = yaml.safe_load(target_config.read_text(encoding="utf-8"))
    with np.load(source, allow_pickle=False) as z:
        q = z["qpos"][-1].copy(); v = z["qvel"][-1].copy()
        sample = target_from_config(cfg).sample(float(z["capture_time_s"]))
        p_error = np.linalg.norm(z["target_center_position"][-1]-sample.center_position_world_m)
        r_error = np.linalg.norm(z["target_center_rotation"][-1]-sample.center_rotation_world)
        if max(p_error, r_error) > 2e-12:
            raise RuntimeError("C1 target analytic state differs from recorded state")
    base_q, base_v = joint_slices(model, "base_free_joint")
    target_q, target_v = joint_slices(model, "target_free_joint")
    data.qpos[base_q] = q[:7]; data.qvel[base_v] = v[:6]
    for i in range(1, 8):
        qslice, vslice = joint_slices(model, f"joint{i}")
        data.qpos[qslice] = q[6+i]
        data.qvel[vslice] = v[5+i]
    quat = Rotation.from_matrix(sample.center_rotation_world).as_quat()
    data.qpos[target_q] = np.r_[sample.center_position_world_m, quat[3], quat[:3]]
    data.qvel[target_v] = np.r_[sample.center_linear_velocity_world_m_s,
                                sample.center_rotation_world.T @ sample.angular_velocity_world_rad_s]
    data.ctrl[:] = 0
    data.time = 0
    mujoco.mj_forward(model, data)
    return {"source_trace_sha256": digest(source), "capture_time_s": sample.time_s,
            "source_target_position_error_m": float(p_error),
            "source_target_rotation_matrix_error": float(r_error),
            "source_joint_qpos": q[7:].tolist(), "source_joint_qvel": v[6:].tolist(),
            "source_base_qpos": q[:7].tolist(), "source_base_qvel": v[:6].tolist()}


def mass_matrix(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    matrix = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, matrix, data.qM)
    return matrix


def equality_rows(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    eid = object_id(model, mujoco.mjtObj.mjOBJ_EQUALITY, name)
    rows = np.flatnonzero((np.asarray(data.efc_type) == int(mujoco.mjtConstraint.mjCNSTR_EQUALITY)) &
                          (np.asarray(data.efc_id) == eid))
    if len(rows) != 6:
        raise RuntimeError(f"expected six named weld rows, got {len(rows)}")
    return rows


def constraint_jacobian(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    if mujoco.mj_isSparse(model):
        matrix = np.zeros((data.nefc, model.nv))
        for row in range(data.nefc):
            start = data.efc_J_rowadr[row]
            size = data.efc_J_rownnz[row]
            matrix[row, data.efc_J_colind[start:start+size]] = data.efc_J[start:start+size]
        return matrix
    return np.asarray(data.efc_J).reshape(data.nefc, model.nv).copy()


def project_velocity(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> dict:
    rows = equality_rows(model, data, name)
    J = constraint_jacobian(model, data)[rows]
    M = mass_matrix(model, data)
    before = data.qvel.copy()
    residual = J @ before
    mass_solve = np.linalg.solve(M, J.T)
    multipliers = np.linalg.solve(J @ mass_solve, residual)
    after = before - mass_solve @ multipliers
    return {"before": before, "after": after, "before_residual": residual,
            "after_residual": J @ after, "kinetic_loss_j":
            float(0.5*before@M@before-0.5*after@M@after)}


def body_jacobian(model: mujoco.MjModel, data: mujoco.MjData, body: int, point: np.ndarray) -> np.ndarray:
    jp, jr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    mujoco.mj_jac(model, data, jp, jr, point, body)
    return np.vstack((jp, jr))


def momenta(model: mujoco.MjModel, data: mujoco.MjData) -> dict:
    masses = np.asarray(model.body_mass)
    total_mass = float(np.sum(masses))
    positions = np.asarray(data.xipos)
    center = np.sum(masses[:, None]*positions, axis=0)/total_mass
    P, H, Ilock = np.zeros(3), np.zeros(3), np.zeros((3,3))
    details = {}
    for body in range(1, model.nbody):
        mass = float(masses[body])
        if not mass:
            continue
        jac = body_jacobian(model, data, body, positions[body])
        velocity = jac @ data.qvel
        linear, angular = velocity[:3], velocity[3:]
        R = np.asarray(data.ximat[body]).reshape(3,3)
        I = R @ np.diag(model.body_inertia[body]) @ R.T
        radius = positions[body]-center
        P += mass*linear
        H += I@angular + np.cross(radius, mass*linear)
        Ilock += I + mass*((radius@radius)*np.eye(3)-np.outer(radius,radius))
        details[mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body)] = {
            "linear_velocity_world_m_s": linear.tolist(),
            "angular_velocity_world_rad_s": angular.tolist()}
    M = mass_matrix(model, data)
    T = float(0.5*data.qvel@M@data.qvel)
    locked = np.linalg.solve(Ilock, H)
    relative_energy = T - float(P@P)/(2*total_mass) - 0.5*float(H@locked)
    return {"mass_kg": total_mass, "center_world_m": center.tolist(),
            "linear_momentum_world_kg_m_s": P.tolist(),
            "angular_momentum_about_center_world_kg_m2_s": H.tolist(),
            "locked_inertia_world_kg_m2": Ilock.tolist(),
            "omega_locked_prediction_world_rad_s": locked.tolist(),
            "kinetic_energy_j": T, "relative_energy_j": float(relative_energy),
            "body_velocities": details}


def interface_observation(model: mujoco.MjModel, data: mujoco.MjData, name: str,
                          tool_site: str, target_site: str) -> dict:
    rows = equality_rows(model, data, name)
    J = constraint_jacobian(model, data)[rows]
    lam = np.asarray(data.efc_force)[rows]
    generalized = J.T @ lam
    ts = object_id(model, mujoco.mjtObj.mjOBJ_SITE, target_site)
    fs = object_id(model, mujoco.mjtObj.mjOBJ_SITE, tool_site)
    point = np.asarray(data.site_xpos[ts]).copy()
    jt = body_jacobian(model, data, int(model.site_bodyid[ts]), point)
    jf = body_jacobian(model, data, int(model.site_bodyid[fs]), point)
    _, target_v = joint_slices(model, "target_free_joint")
    wrench_world = np.linalg.solve(jt[:, target_v].T, generalized[target_v])
    wrench_robot_world = np.linalg.lstsq(jf[:, target_v.stop:].T,
                                         generalized[target_v.stop:], rcond=None)[0]
    R = np.asarray(data.site_xmat[ts]).reshape(3,3)
    wrench_grasp = np.r_[R.T@wrench_world[:3], R.T@wrench_world[3:]]
    f_norm, m_norm = np.linalg.norm(wrench_grasp[:3]), np.linalg.norm(wrench_grasp[3:])
    Rf = np.asarray(data.site_xmat[fs]).reshape(3,3)
    angle = Rotation.from_matrix(Rf.T@R).magnitude()
    tool_j = body_jacobian(model, data, int(model.site_bodyid[fs]), np.asarray(data.site_xpos[fs]))
    target_j = body_jacobian(model, data, int(model.site_bodyid[ts]), point)
    return {"force_grasp_n": wrench_grasp[:3].tolist(),
            "moment_grasp_nm": wrench_grasp[3:].tolist(),
            "force_norm_n": float(f_norm), "moment_norm_nm": float(m_norm),
            "load_fraction": float(f_norm/50 + m_norm/2),
            "force_target_world_n": wrench_world[:3].tolist(),
            "moment_target_world_nm": wrench_world[3:].tolist(),
            "action_reaction_residual_world": (wrench_world+wrench_robot_world).tolist(),
            "generalized_reconstruction_error": float(np.linalg.norm(generalized -
                jt.T@wrench_world - jf.T@wrench_robot_world)),
            "translation_error_m": float(np.linalg.norm(data.site_xpos[ts]-data.site_xpos[fs])),
            "rotation_error_deg": float(np.rad2deg(angle)),
            "relative_twist_world": ((target_j-tool_j)@data.qvel).tolist(),
            "constraint_power_w": float(lam@(J@data.qvel)),
            "efc_row_indices": rows.tolist()}


def damping_coefficients(model: mujoco.MjModel, data: mujoco.MjData,
                         tool_site: str, target_site: str, time_scale: float) -> dict:
    fs = object_id(model, mujoco.mjtObj.mjOBJ_SITE, tool_site)
    ts = object_id(model, mujoco.mjtObj.mjOBJ_SITE, target_site)
    point = np.asarray(data.site_xpos[ts])
    jf = body_jacobian(model, data, int(model.site_bodyid[fs]), point)
    jt = body_jacobian(model, data, int(model.site_bodyid[ts]), point)
    _, target_v = joint_slices(model, "target_free_joint")
    # Coordinates are [target free, base free, seven named joints].  Preserve
    # actual joint coordinates when eliminating the six unconstrained base DOFs.
    base = np.arange(*joint_slices(model, "base_free_joint")[1].indices(model.nv))
    joints = np.array([joint_slices(model, f"joint{i}")[1].start for i in range(1,8)])
    robot = np.r_[base, joints]
    N = np.zeros((model.nv, 13))
    N[robot, np.arange(13)] = 1
    N[target_v, :] = -np.linalg.solve(jt[:, target_v], jf[:, robot])
    reduced = N.T @ mass_matrix(model, data) @ N
    effective = reduced[6:,6:] - reduced[6:,:6] @ np.linalg.solve(reduced[:6,:6], reduced[:6,6:])
    symmetry = np.linalg.norm(effective-effective.T)
    eig = np.linalg.eigvalsh((effective+effective.T)/2)
    if symmetry > 1e-9 or np.min(eig) <= 0:
        raise RuntimeError("effective joint inertia is not symmetric positive definite")
    return {"M_eff_kg_m2": effective.tolist(), "coefficient_nm_s_rad":
            (np.diag(effective)/time_scale).tolist(),
            "minimum_eigenvalue": float(np.min(eig)), "symmetry_error": float(symmetry),
            "method": "physical_jacobian_target_elimination_then_base_schur_complement"}
