"""Finite free-body interface tests with balanced, smoothly applied wrenches."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from v6_mujoco.postgrasp.physics import (body_jacobian, digest, load_model,
    mass_matrix, momenta, object_id, interface_observation)
from .common import ROOT, OLD, MODEL, NAME, read, save, pose_reference, pose, frames, runtime_contract, solver_diagnostics


def fmt(x):return " ".join(f"{float(v):.17g}" for v in x)


def quat(R):
    q=Rotation.from_matrix(R).as_quat()
    return np.r_[q[3],q[:3]]


def fixture(parameters, timestep=.002):
    formal=load_model(MODEL)
    state=read(OLD/"initial_state.json")
    data=mujoco.MjData(formal);data.qpos[:]=state["qpos_after"]
    mujoco.mj_forward(formal,data)
    target=object_id(formal,mujoco.mjtObj.mjOBJ_BODY,"tumbling_target")
    distal=object_id(formal,mujoco.mjtObj.mjOBJ_BODY,"link7")
    sid=object_id(formal,mujoco.mjtObj.mjOBJ_SITE,"target_grasp_site")
    Pt,Pr=data.xipos[target].copy(),data.xipos[distal].copy()
    Rt,Rr=data.ximat[target].reshape(3,3).copy(),data.ximat[distal].reshape(3,3).copy()
    Ps=data.site_xpos[sid].copy();Rs=data.site_xmat[sid].reshape(3,3).copy()
    relative=np.r_[Rr.T@(Pt-Pr),quat(Rr.T@Rt)]
    geometry={"target_mass_kg":float(formal.body_mass[target]),
              "target_inertia_kg_m2":formal.body_inertia[target].tolist(),
              "tool_mass_kg":float(formal.body_mass[distal]),
              "tool_inertia_kg_m2":formal.body_inertia[distal].tolist(),
              "source_tool_body":"link7 inertial frame; full upstream arm excluded in isolated fixture",
              "source_model_sha256":digest(MODEL),"initial_interface_point_world_m":Ps.tolist(),
              "tool_pad_diameter_m":.07,"target_grasp_offset_m":.15}
    xml=f'''<mujoco model="n201r_free_interface_fixture">
      <compiler angle="radian"/>
      <option timestep="{timestep}" gravity="0 0 0" integrator="RK4" iterations="50" tolerance="1e-10"/>
      <worldbody>
        <body name="tumbling_target" pos="{fmt(Pt)}" quat="{fmt(quat(Rt))}">
          <freejoint name="target_free_joint"/>
          <inertial pos="0 0 0" mass="{geometry['target_mass_kg']}" diaginertia="{fmt(geometry['target_inertia_kg_m2'])}"/>
          <site name="target_grasp_site" pos="{fmt(Rt.T@(Ps-Pt))}" quat="{fmt(quat(Rt.T@Rs))}"/>
          <geom type="box" size=".15 .15 .15" density="0" contype="0" conaffinity="0"/>
        </body>
        <body name="flange" pos="{fmt(Pr)}" quat="{fmt(quat(Rr))}">
          <freejoint name="tool_free_joint"/>
          <inertial pos="0 0 0" mass="{geometry['tool_mass_kg']}" diaginertia="{fmt(geometry['tool_inertia_kg_m2'])}"/>
          <site name="postgrasp_tool_interface" pos="{fmt(Rr.T@(Ps-Pr))}" quat="{fmt(quat(Rr.T@Rs))}"/>
          <geom type="sphere" size=".035" density="0" contype="0" conaffinity="0"/>
        </body>
      </worldbody>
      <equality><weld name="postgrasp_latch" body1="flange" body2="tumbling_target"
        relpose="{fmt(relative)}" solref="{fmt(parameters['solref'])}" solimp="{fmt(parameters['solimp'])}"
        torquescale="{parameters['torquescale_m']}"/></equality>
    </mujoco>'''
    return mujoco.MjModel.from_xml_string(xml),geometry,xml


def manifest():
    path=ROOT/"load_test_manifest.json"
    if path.exists():return read(path)
    cases=[]
    for axis in range(6):
        for sign in (-1,1):
            wrench=np.zeros(6);wrench[axis]=sign*(50 if axis<3 else 2)
            cases.append({"name":f"{'force' if axis<3 else 'moment'}_{'xyz'[axis%3]}_{'negative' if sign<0 else 'positive'}",
                          "wrench_grasp":wrench.tolist()})
    for index,values in enumerate(([25/np.sqrt(2),25/np.sqrt(2),0,0,0,1],
                                  [0,0,-25,1/np.sqrt(2),-1/np.sqrt(2),0],
                                  [-25,0,0,0,1,0], [0,25,0,-1,0,0])):
        cases.append({"name":f"mixed_{index}","wrench_grasp":values})
    payload={"schema_version":"n201r_frozen_load_test_v1","cases":cases,
        "profile":{"ramp_s":.2,"hold_end_s":.5,"unload_end_s":.7,"end_s":.9,
                   "shape":"quintic smoothstep; held during each RK4 step"},
        "input_reference":"current target grasp frame and point; both sides shifted to their COM",
        "gates":{"translation_m":.0005,"rotation_deg":.1,"linear_momentum_drift":1e-4,
                 "angular_momentum_drift":1e-5,"equality_kkt_acceleration_residual":1e-7,
                 "balanced_net_force_moment":1e-10},
        "step_sensitivity_gates":{"translation_error_m":.00005,"rotation_error_deg":.01},
        "load_envelope":{"force_n":50,"moment_nm":2,"provenance":"simulation_assumption"},
        "approximation":"two free rigid bodies, actual target and actual distal link7 inertias; excludes upstream arm flexibility",
        "formal_initial_mobility_is_also_used_for_parameter_derivation":True,
        "candidate_budget":2,"parameter_selection_uses_detumbling":False}
    save(path,payload)
    save(ROOT/"run_ledger.json",{"formal_runs":[],"interface_tests":[],"independent_replays":[],
                                "fixed_state_audits":["historical pose/initialization/solver"],
                                "complete_robot_dynamics_attempts":[]})
    return payload


def profile(t):
    smooth=lambda x:10*x**3-15*x**4+6*x**5
    if t<.2:return smooth(max(0,t/.2))
    if t<=.5:return 1.0
    if t<.7:return 1-smooth((t-.5)/.2)
    return 0.0


def test_case(parameters, case, timestep):
    model,geometry,xml=fixture(parameters,timestep)
    data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    ref=pose_reference(model,data)
    target=object_id(model,mujoco.mjtObj.mjOBJ_BODY,"tumbling_target")
    tool=object_id(model,mujoco.mjtObj.mjOBJ_BODY,"flange")
    sid=object_id(model,mujoco.mjtObj.mjOBJ_SITE,"target_grasp_site")
    records=[]
    for step in range(int(round(.9/timestep))+1):
        R=np.asarray(data.site_xmat[sid]).reshape(3,3);point=data.site_xpos[sid].copy()
        commanded=np.asarray(case["wrench_grasp"])*profile(data.time)
        world=np.r_[R@commanded[:3],R@commanded[3:]]
        F,M=world[:3],world[3:]
        data.xfrc_applied[target]=np.r_[F,M+np.cross(point-data.xipos[target],F)]
        data.xfrc_applied[tool]=np.r_[-F,-M+np.cross(point-data.xipos[tool],-F)]
        mujoco.mj_forward(model,data)
        current=pose(model,data,ref)
        face=interface_observation(model,data,NAME,"postgrasp_tool_interface","target_grasp_site")
        physics=momenta(model,data);solver=solver_diagnostics(model,data)
        jt=body_jacobian(model,data,target,point);jr=body_jacobian(model,data,tool,point)
        netF=data.xfrc_applied[target,:3]+data.xfrc_applied[tool,:3]
        netM=sum((data.xfrc_applied[b,3:]+np.cross(data.xipos[b],data.xfrc_applied[b,:3]) for b in (target,tool)),np.zeros(3))
        records.append({"time_s":float(data.time),"qpos":data.qpos.copy(),"qvel":data.qvel.copy(),
           "translation_error_m":current["translation_error_m"],"rotation_error_deg":current["rotation_error_deg"],
           "translation_error_tool_m":current["translation_error_tool_m"],
           "commanded_wrench_grasp":commanded,"interface_wrench_grasp":np.r_[face["force_grasp_n"],face["moment_grasp_nm"]],
           "interface_load_fraction":face["load_fraction"],"net_force_moment":np.r_[netF,netM],
           "P":physics["linear_momentum_world_kg_m_s"],"H":physics["angular_momentum_about_center_world_kg_m2_s"],
           "kinetic_energy_j":physics["kinetic_energy_j"],"external_power_w":float(world@((jt-jr)@data.qvel)),
           "constraint_power_w":face["constraint_power_w"],"solver_residual":solver["equality_kkt_max_absolute"],
           "constraint_position_residual":solver["efc_pos"],"solver_iterations":int(np.max(data.solver_niter))})
        if step==int(round(.9/timestep)):break
        mujoco.mj_step(model,data)
    arrays={k:np.asarray([r[k] for r in records]) for k in records[0]}
    t=arrays["time_s"];hold=(t>=.3-1e-12)&(t<=.5+1e-12)
    gate=manifest()["gates"]
    maxp=float(np.max(arrays["translation_error_m"]));maxa=float(np.max(arrays["rotation_error_deg"]))
    Pd=float(np.max(np.linalg.norm(arrays["P"]-arrays["P"][0],axis=1)))
    Hd=float(np.max(np.linalg.norm(arrays["H"]-arrays["H"][0],axis=1)))
    balanced=float(np.max(np.linalg.norm(arrays["net_force_moment"],axis=1)))
    residual=float(np.max(arrays["solver_residual"]))
    result={"case":case["name"],"timestep_s":timestep,"parameters":parameters,
        "maximum_translation_m":maxp,"maximum_rotation_deg":maxa,
        "hold_max_translation_m":float(np.max(arrays["translation_error_m"][hold])),
        "hold_max_rotation_deg":float(np.max(arrays["rotation_error_deg"][hold])),
        "max_interface_load_fraction":float(np.max(arrays["interface_load_fraction"])),
        "hold_mean_wrench_grasp":np.mean(arrays["interface_wrench_grasp"][hold],axis=0).tolist(),
        "hold_mean_translation_tool_m":np.mean(arrays["translation_error_tool_m"][hold],axis=0).tolist(),
        "linear_momentum_drift":Pd,"angular_momentum_drift":Hd,
        "maximum_net_force_moment":balanced,"maximum_solver_residual":residual,
        "maximum_solver_iterations":int(np.max(arrays["solver_iterations"])),
        "external_work_j":float(np.trapz(arrays["external_power_w"],t)),
        "constraint_work_j":float(np.trapz(arrays["constraint_power_w"],t)),
        "kinetic_energy_change_j":float(arrays["kinetic_energy_j"][-1]-arrays["kinetic_energy_j"][0]),
        "pose_holding_passed":maxp<=gate["translation_m"] and maxa<=gate["rotation_deg"],
        "numerics_passed":Pd<=gate["linear_momentum_drift"] and Hd<=gate["angular_momentum_drift"] and
                          balanced<=gate["balanced_net_force_moment"] and residual<=gate["equality_kkt_acceleration_residual"]}
    result["status"]="QUALIFIED" if result["pose_holding_passed"] and result["numerics_passed"] else "MODEL_COMPLIANCE_MISMATCH" if not result["pose_holding_passed"] else "NUMERICAL_TEST_FAILURE"
    return result,arrays,geometry,xml


def run_suite(label,parameters,timestep=.002):
    spec=manifest();location=ROOT/"load_tests"/label
    if location.exists():raise FileExistsError(f"load suite already exists: {label}")
    location.mkdir(parents=True)
    results=[]
    for case in spec["cases"]:
        result,arrays,geometry,xml=test_case(parameters,case,timestep)
        np.savez_compressed(location/(case["name"]+".npz"),**arrays)
        results.append(result)
    (location/"fixture.xml").write_text(xml,encoding="utf-8")
    payload={"label":label,"geometry":geometry,"parameters":parameters,"timestep_s":timestep,
             "results":results,"all_passed":all(r["status"]=="QUALIFIED" for r in results)}
    save(location/"results.json",payload)
    path=ROOT/"load_test_results.json";all_results=read(path) if path.exists() else {"suites":{}}
    all_results["suites"][label]=payload;save(path,all_results)
    ledger=read(ROOT/"run_ledger.json");ledger["interface_tests"].append({"suite":label,"cases":len(results),
              "duration_each_s":.9,"timestep_s":timestep,"all_passed":payload["all_passed"]})
    save(ROOT/"run_ledger.json",ledger)
    print(json.dumps({"suite":label,"all_passed":payload["all_passed"],
                      "max_translation_m":max(r["maximum_translation_m"] for r in results),
                      "max_rotation_deg":max(r["maximum_rotation_deg"] for r in results)}))
    return payload


def derive_candidate():
    original=read(ROOT/"load_test_results.json")["suites"]["original"]
    if original["all_passed"]:raise RuntimeError("original connection already qualifies; calibration unnecessary")
    old={"solref":[.02,1],"solimp":[.95,.99,.001,.5,2],"torquescale_m":.15}
    fixture_model,geometry,_=fixture(old)
    fdata=mujoco.MjData(fixture_model);mujoco.mj_forward(fixture_model,fdata)
    formal=load_model(MODEL);data=mujoco.MjData(formal)
    state=read(OLD/"initial_state.json");data.qpos[:]=state["qpos_after"];mujoco.mj_forward(formal,data)
    bounds={}
    for label,m,d in (("distal_fixture",fixture_model,fdata),("formal_initial_configuration",formal,data)):
        sid=object_id(m,mujoco.mjtObj.mjOBJ_SITE,"target_grasp_site")
        tool=object_id(m,mujoco.mjtObj.mjOBJ_BODY,"flange");target=object_id(m,mujoco.mjtObj.mjOBJ_BODY,"tumbling_target")
        point=d.site_xpos[sid]
        J=body_jacobian(m,d,target,point)-body_jacobian(m,d,tool,point)
        mobility=J@np.linalg.solve(mass_matrix(m,d),J.T)
        # The weighted L2-sum envelope is a convex hull of the force and moment
        # balls; its maximum acceleration norm is bounded by these operator norms.
        av=max(50*np.linalg.norm(mobility[:3,:3],2),2*np.linalg.norm(mobility[:3,3:],2))
        aw=max(50*np.linalg.norm(mobility[3:,:3],2),2*np.linalg.norm(mobility[3:,3:],2))
        bounds[label]={"relative_mobility":mobility.tolist(),"linear_acceleration_bound_m_s2":float(av),
                       "angular_acceleration_bound_rad_s2":float(aw)}
    av=max(v["linear_acceleration_bound_m_s2"] for v in bounds.values())
    aw=max(v["angular_acceleration_bound_rad_s2"] for v in bounds.values())
    timeconst=.008; margin=4.0
    deficit=min(.0005/(margin*timeconst**2*av),np.deg2rad(.1)/(margin*timeconst**2*aw))
    impedance=1-deficit
    if impedance>=.9999 or impedance<=0:raise RuntimeError("derived finite impedance outside supported interval")
    parameters={"solref":[timeconst,1.0],"solimp":[impedance,impedance,.001,.5,2],"torquescale_m":.07}
    result={"candidate":"candidate_1","parameters":parameters,"derivation":{
        "formula":"r_static ~= a_unconstrained * (1-d) * timeconst^2; constant impedance; margin=4",
        "timeconst_choice":"8 ms = four formal steps; above refsafe lower bound at both step sizes",
        "torquescale_choice":"70 mm = existing tool pad diameter; physical SO(3) gate is unchanged",
        "mobility_bounds":bounds,"deficit_1_minus_d":deficit,"design_margin":margin,
        "quasistatic_design_translation_stiffness_n_m":100000,
        "quasistatic_design_angular_stiffness_nm_rad":float(2/np.deg2rad(.1)),
        "parameters_are_acceleration_normalized_not_physical_spring_constants":True},
        "qualification_status":"PENDING_INDEPENDENT_TESTS","candidate_count":1}
    save(ROOT/"candidate_1.json",result)
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument("phase",choices=("original","derive","candidate","fine"))
    args=parser.parse_args()
    if args.phase=="original":run_suite("original",{"solref":[.02,1],"solimp":[.95,.99,.001,.5,2],"torquescale_m":.15})
    elif args.phase=="derive":print(json.dumps(derive_candidate()["parameters"]))
    elif args.phase=="candidate":run_suite("candidate_1",read(ROOT/"candidate_1.json")["parameters"])
    else:run_suite("candidate_1_fine",read(ROOT/"candidate_1.json")["parameters"],.001)


if __name__=="__main__":main()
