"""Zero new trajectories: re-evaluate historical states and frozen references."""

import json
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from v6_mujoco.postgrasp.physics import (digest, load_model, equality_rows,
    constraint_jacobian, interface_observation, momenta)
from v6_mujoco.postgrasp.run import read_config, known_wrench_test
from .common import ROOT, OLD, MODEL, NAME, read, save, pose_reference, pose, relative_rotation_error, runtime_contract, solver_diagnostics


def main():
    cfg = read_config()
    manifest = read(OLD/"experiment_manifest.json")
    for path, expected in manifest["implementation_identity"].items():
        from v6_mujoco.model import PROJECT_ROOT
        if digest(PROJECT_ROOT/path) != expected:
            raise RuntimeError(f"historical identity mismatch {path}")
    old_files = ["experiment_manifest.json","model_contract.json","initial_state.json",
                 "comparison.json","validation.json","damping.json","wrench_unit_tests.json"]
    old_files += [f"{c}/{f}" for c in ("Z","D","D_fine") for f in ("trace.npz","metrics.json","termination.json")]
    inspected = {f: digest(OLD/f) for f in old_files}
    for f in old_files:
        if f.endswith(".json"):
            read(OLD/f)
    model = load_model(MODEL)
    data = mujoco.MjData(model)
    initial = read(OLD/"initial_state.json")
    data.qpos[:] = initial["qpos_after"]; data.qvel[:] = initial["qvel_after"]
    mujoco.mj_forward(model,data)
    reference = pose_reference(model,data)
    rows = equality_rows(model,data,NAME)
    J = constraint_jacobian(model,data)[rows]
    before = initial["before_physics"]
    after = momenta(model,data)
    init_audit = {"physical_pose":pose(model,data,reference),
        "position_residual":data.efc_pos[rows].tolist(),"velocity_residual":(J@data.qvel).tolist(),
        "row_count":len(rows),"rank":int(np.linalg.matrix_rank(J)),
        "recomputed_momentum_after":after,
        "recorded_projection_momentum_changes":{k:initial[k] for k in
            ("linear_momentum_change_kg_m_s","angular_momentum_change_kg_m2_s","kinetic_loss_j",
             "target_omega_before_deg_s","target_omega_after_deg_s")}}
    # A deliberately non-identity frozen relative orientation exercises the
    # reference subtraction, while common world rotations must cancel.
    RA=Rotation.from_euler("xyz",[.2,-.3,.1]).as_matrix()
    frozen=Rotation.from_rotvec([.1,.2,-.15]).as_matrix(); RB=RA@frozen
    Q=Rotation.from_rotvec([.7,-.8,.3]).as_matrix()
    small=Rotation.from_rotvec([0,0,np.deg2rad(.05)]).as_matrix()
    q=Rotation.from_matrix(RB).as_quat()
    tests={"common_rotation_rad":relative_rotation_error(Q@RA,Q@RB,frozen),
           "initial_nonidentity_reference_rad":relative_rotation_error(RA,RB,frozen),
           "quaternion_sign_difference_rad":abs(relative_rotation_error(RA,Rotation.from_quat(q).as_matrix(),frozen)-
              relative_rotation_error(RA,Rotation.from_quat(-q).as_matrix(),frozen)),
           "known_small_angle_error_rad":abs(relative_rotation_error(RA,RB@small,frozen)-np.deg2rad(.05))}
    if max(tests.values())>1e-12 or init_audit["rank"]!=6 or np.linalg.norm(J@data.qvel)>1e-10:
        raise RuntimeError("pose definition or initialization audit failed")
    failures={}
    for c in ("Z","D","D_fine"):
        model.opt.timestep=read(OLD/c/"metrics.json")["timestep_s"]
        with np.load(OLD/c/"trace.npz",allow_pickle=False) as trace:
            physical=[]
            for qpos,qvel in zip(trace["qpos"],trace["qvel"]):
                data.qpos[:]=qpos; data.qvel[:]=qvel
                mujoco.mj_forward(model,data)
                physical.append(pose(model,data,reference)["rotation_error_deg"])
            data.qpos[:]=trace["qpos"][-1];data.qvel[:]=trace["qvel"][-1];data.ctrl[:]=trace["ctrl_nm"][-1]
            data.qacc_warmstart[:]=0; mujoco.mj_forward(model,data)
            diagnostics=solver_diagnostics(model,data)
            face=interface_observation(model,data,NAME,"postgrasp_tool_interface","target_grasp_site")
            gradient_original=diagnostics["equality_kkt_max_absolute"]
            model.opt.iterations=100;model.opt.tolerance=1e-12
            mujoco.mj_forward(model,data)
            stricter=solver_diagnostics(model,data)
            model.opt.iterations=50;model.opt.tolerance=1e-10
            failures[c]={"time_s":float(trace["time_s"][-1]),"physical_pose":pose(model,data,reference),
                "old_field_deg":float(trace["interface_rotation_error_deg"][-1]),
                "all_samples_max_angle_difference_deg":float(np.max(np.abs(np.array(physical)-trace["interface_rotation_error_deg"]))),
                "relative_twist_world":face["relative_twist_world"],"interface_wrench":face,
                "joint_torque_nm":trace["ctrl_nm"][-1].tolist(),"solver":diagnostics,
                "runtime_contract":runtime_contract(model,data),
                "fixed_state_stricter_solver_residual":stricter["equality_kkt_max_absolute"],
                "forward_reconstruction_warmstart":"zero; fixed-state audit, no trajectory"}
    model.opt.timestep=.002
    data.qpos[:]=initial["qpos_after"];data.qvel[:]=initial["qvel_after"]
    data.ctrl[:]=0;mujoco.mj_forward(model,data)
    contract=runtime_contract(model,data)
    known=known_wrench_test(model,data,cfg)
    if not contract["active"] or contract["definition"]!="body_based" or not contract["refsafe"]:
        raise RuntimeError("unexpected active weld contract")
    result={"schema_version":"n201r_pose_audit_v1","historical_artifacts_inspected":inspected,
      "formula":"Log(Rrel0.T @ RA.T @ RB)","reference_frozen":True,
      "reference_rotation":reference["rotation"].tolist(),"reference_translation_m":reference["translation"].tolist(),
      "tests":tests,"initialization":init_audit,"historical_failures":failures,
      "observation_passed":True,"observation_error_found":False,"initialization_error_found":False,
      "reused_known_wrench_test":known,"prior_test_gap":"no 0.1 deg loaded holding test; prior translation bound was 2 mm",
      "prior_checks_reused":["named equality wrench","collision mask audit","strict replay evidence"],
      "short_archived_runs_complete_10s":False}
    save(ROOT/"pose_definition_audit.json",result)
    save(ROOT/"runtime_equality_contract.json",contract)
    print(json.dumps({"observation_passed":True,"initialization_rank":int(np.linalg.matrix_rank(J)),
                     "failure_angle_differences":{c:r["all_samples_max_angle_difference_deg"] for c,r in failures.items()}}))


if __name__=="__main__":main()
