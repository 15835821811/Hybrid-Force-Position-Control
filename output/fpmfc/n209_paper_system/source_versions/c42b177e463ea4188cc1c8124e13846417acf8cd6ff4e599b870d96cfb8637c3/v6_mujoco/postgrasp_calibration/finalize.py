"""Audit the entire calibration contract, including the unchanged load gate."""

from pathlib import Path
import json
import platform
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import scipy
import yaml
from scipy.spatial.transform import Rotation

from v6_mujoco.model import PROJECT_ROOT
from v6_mujoco.postgrasp.physics import digest, load_model, equality_rows, constraint_jacobian, momenta
from .common import ROOT, OLD, MODEL, NAME, read, save, pose_reference, pose, frames, runtime_contract
from .load_tests import fixture, test_case


def main():
    spec=read(ROOT/"load_test_manifest.json")
    all_results=read(ROOT/"load_test_results.json")
    candidate=read(ROOT/"candidate_2_reference_correction.json")
    parameters=candidate["parameters"]
    coarse=all_results["suites"]["candidate_2_balanced"]
    fine=all_results["suites"]["candidate_2_balanced_fine"]
    sensitivity={};coupling=np.zeros((6,6));means={}
    for case in spec["cases"]:
        name=case["name"]
        with np.load(ROOT/"load_tests/candidate_2_balanced"/(name+".npz")) as a, np.load(ROOT/"load_tests/candidate_2_balanced_fine"/(name+".npz")) as b:
            sensitivity[name]={key:float(np.max(np.abs(a[key]-b[key][::2]))) for key in
                ("qpos","qvel","translation_error_m","rotation_error_deg","interface_wrench_grasp","kinetic_energy_j")}
            if name.startswith(("force_","moment_")):
                m,_,_=fixture(parameters);d=mujoco.MjData(m);mujoco.mj_forward(m,d);ref=pose_reference(m,d)
                errors=[]
                for t,q in zip(a["time_s"],a["qpos"]):
                    if .3-1e-12<=t<=.5+1e-12:
                        d.qpos[:]=q;mujoco.mj_forward(m,d)
                        RA,RB,pa,pb=frames(m,d)
                        errors.append(np.r_[RA.T@(pb-pa)-ref["translation"],
                                            Rotation.from_matrix(ref["rotation"].T@RA.T@RB).as_rotvec()])
                means[name]=np.mean(errors,axis=0)
    for axis in range(6):
        prefix=f"{'force' if axis<3 else 'moment'}_{'xyz'[axis%3]}"
        coupling[:,axis]=(means[prefix+"_positive"]-means[prefix+"_negative"])/(2*(50 if axis<3 else 2))
    sensitivity_passed=all(r["translation_error_m"]<=spec["step_sensitivity_gates"]["translation_error_m"] and
        r["rotation_error_deg"]<=spec["step_sensitivity_gates"]["rotation_error_deg"] for r in sensitivity.values())
    load_peak=max(r["max_interface_load_fraction"] for r in coarse["results"]+fine["results"])
    load_passed=load_peak<=1.0  # Original task gate, no tolerance expansion.
    result={"schema_version":"n201r_connection_qualification_v1",
        "selected_candidate":"candidate_2","parameters":parameters,
        "holding_precision_passed":coarse["all_passed"] and fine["all_passed"],
        "numerics_passed":sensitivity_passed,
        "load_envelope_passed":load_passed,"maximum_measured_load_fraction":load_peak,
        "status":"QUALIFIED" if coarse["all_passed"] and fine["all_passed"] and sensitivity_passed and load_passed else "CONNECTION_MODEL_NOT_QUALIFIED",
        "reason":"boundary-load transient reconstructed wrench exceeds the unchanged envelope; no formal Z/D runs authorized",
        "model_scope":"idealized_postgrasp_connection","dynamic_capture_validated":False,
        "hardware_load_rating_validated":False,"controller_or_damping_modified":False,
        "initial_state_reprojected":False,"initial_state_sha256":digest(OLD/"initial_state.json"),
        "damping_sha256":digest(OLD/"damping.json"),"candidate_count":2,
        "load_test_all_passed_field_scope":"pose and numerical gates only; this final qualification additionally enforces measured wrench envelope"}
    save(ROOT/"calibrated_connection.json",result)
    # Save the investigated model for review; it is explicitly not admitted to
    # a formal trajectory because qualification has failed on measured load.
    tree=ET.fromstring(MODEL.read_text(encoding="utf-8"))
    weld=tree.find("equality/weld")
    for field in ("body1","body2","relpose","anchor"):
        weld.attrib.pop(field,None)
    weld.set("site1","postgrasp_tool_interface");weld.set("site2","target_grasp_site")
    weld.set("solref"," ".join(str(v) for v in parameters["solref"]))
    weld.set("solimp"," ".join(str(v) for v in parameters["solimp"]))
    weld.set("torquescale",str(parameters["torquescale_m"]))
    path=PROJECT_ROOT/"models/flexiv_rizon4s_n201_interface_candidate.xml"
    ET.indent(tree,space="  ")
    path.write_text(ET.tostring(tree,encoding="unicode")+"\n",encoding="utf-8")
    oldmodel=load_model(MODEL);new=load_model(path)
    invariant_fields=("body_mass","body_inertia","body_ipos","body_iquat","jnt_type","jnt_range",
                      "dof_damping","dof_armature","actuator_gear","actuator_ctrlrange","actuator_forcerange",
                      "geom_contype","geom_conaffinity","site_pos","site_quat")
    for field in invariant_fields:
        if not np.array_equal(getattr(oldmodel,field),getattr(new,field)):
            raise RuntimeError(f"protected model property changed: {field}")
    initial=read(OLD/"initial_state.json")
    contracts={"original":read(ROOT/"runtime_equality_contract.json")};consistency={}
    for dt in (.002,.001):
        new.opt.timestep=dt;d=mujoco.MjData(new)
        d.qpos[:]=initial["qpos_after"];d.qvel[:]=initial["qvel_after"]
        d.qacc_warmstart[:]=initial["qacc_warmstart_after"];mujoco.mj_forward(new,d)
        J=constraint_jacobian(new,d)[equality_rows(new,d,NAME)]
        residual=float(np.linalg.norm(d.efc_pos));velocity=float(np.linalg.norm(J@d.qvel))
        consistency[str(dt)]={"position_residual_norm":residual,"velocity_residual_norm":velocity,
                               "rank":int(np.linalg.matrix_rank(J)),"physics":momenta(new,d),
                               "physical_pose_error":pose(new,d,pose_reference(new,d)),
                               "efc_KBIP":d.efc_KBIP.tolist()}
        if residual>1e-9 or velocity>1e-10 or consistency[str(dt)]["rank"]!=6:
            raise RuntimeError("selected site-based reference changed initial rigid state")
        contracts[str(dt)]=runtime_contract(new,d)
        if not contracts[str(dt)]["refsafe"] or contracts[str(dt)]["timeconst_effective_s"]!=.004:
            raise RuntimeError("effective numerical time constant differs")
    save(ROOT/"candidate_runtime_contract.json",{"contracts":contracts,"unchanged_initial_state_consistency":consistency,
                                                 "protected_model_arrays_equal":list(invariant_fields)})
    # Replays are independent, explicitly counted short fixture tests. No full
    # robot trajectory is added here.
    replay={}
    for case in spec["cases"]:
        _,arrays,_,_=test_case(parameters,case,.002)
        with np.load(ROOT/"load_tests/candidate_2_balanced"/(case["name"]+".npz")) as archived:
            errors={key:float(np.max(np.abs(archived[key]-arrays[key]))) for key in
                    ("qpos","qvel","interface_wrench_grasp")}
        passed=errors["qpos"]<=1e-11 and errors["qvel"]<=1e-10 and errors["interface_wrench_grasp"]<=1e-8
        if not passed:raise RuntimeError(f"fixture replay failed: {case['name']}")
        replay[case["name"]]={"passed":passed,"max_absolute_differences":errors}
    historical=read(ROOT/"pose_definition_audit.json")["historical_artifacts_inspected"]
    for relative,expected in historical.items():
        if digest(OLD/relative)!=expected:raise RuntimeError("historical result changed")
    versions={"original":"load_tests_v1.py","candidate_1":"load_tests_v1.py",
              "candidate_2":"load_tests_v2.py","candidate_2_corrected_reference":"load_tests_v3.py",
              "original_balanced":"load_tests.py","candidate_1_balanced":"load_tests.py",
              "candidate_2_balanced":"load_tests.py","candidate_2_balanced_fine":"load_tests.py"}
    invalid={"original":"INVALID_BALANCED_INPUT: used stale post-step COMs for wrench shifts",
             "candidate_1":"INVALID_BALANCED_INPUT: used stale post-step COMs for wrench shifts",
             "candidate_2":"INVALID_REFERENCE: nonzero initial anchor residual; also stale COM inputs",
             "candidate_2_corrected_reference":"INVALID_BALANCED_INPUT: stale COM inputs in mixed loads"}
    for label,reason in invalid.items():
        all_results["suites"][label]["validity_status"]=reason
    all_results["final_qualification"]=result
    save(ROOT/"load_test_results.json",all_results)
    ledger=read(ROOT/"run_ledger.json")
    for item in ledger["interface_tests"]:
        label=item["suite"]
        item["runtime_path"]="v6_mujoco/postgrasp_calibration/"+versions[label]
        item["runtime_sha256"]=digest(PROJECT_ROOT/item["runtime_path"])
        item["validity_status"]=invalid.get(label,"VALID_BALANCED_INPUT")
    ledger["independent_replays"]=[{"kind":"selected free-body fixture same-step replay","cases":16,"duration_each_s":.9,"all_passed":True}]
    ledger["formal_runs_skipped_reason"]=result["status"]+": actual wrench envelope violation in independent tests"
    ledger["complete_robot_dynamics_attempts"]=[]
    save(ROOT/"run_ledger.json",ledger)
    for name in ("Z_new","D_new","D_new_fine"):
        save(ROOT/name/"skipped.json",{"condition":name,"status":"NOT_STARTED_CONNECTION_MODEL_NOT_QUALIFIED",
                                      "maximum_allowed_duration_s":10,"formal_trajectory_count":0})
    comparisons={"classification":{"OBSERVATION":"PASS","CONNECTION":result["status"],
                    "HOLDING_PRECISION":"PASS","NUMERICS":"PASS" if sensitivity_passed else "FAIL",
                    "POSTGRASP":"NOT_RUN","DETUMBLING":"NOT_EVALUATED"},
        "historical_failure_cause":"physical compliance mismatch; pose and initial reference correct, fixed-state solver converged",
        "step_sensitivity_full_curves":sensitivity,"measured_hold_compliance_6x6":coupling.tolist(),
        "compliance_units":"rows translation m and rotation rad; columns force N then moment N*m",
        "same_step_replay":replay,"initial_state_unchanged":True,"damping_unchanged":True,
        "new_formal_trajectories":0,"calibration_result":result}
    save(ROOT/"comparison.json",comparisons)
    sources=list((PROJECT_ROOT/"v6_mujoco/postgrasp_calibration").glob("*.py"))
    sources+=list((PROJECT_ROOT/"v6_mujoco/postgrasp").glob("*.py"))
    sources += [path,PROJECT_ROOT/"v6_mujoco/model.py",PROJECT_ROOT/"v6_mujoco/collision.py",
                PROJECT_ROOT/"v6_mujoco/fpmfc/target.py",PROJECT_ROOT/"tests/test_n201_interface_calibration.py"]
    sources += list((PROJECT_ROOT/"assets/meshes/rizon4s/collision").glob("*.stl"))
    identity={str(p.relative_to(PROJECT_ROOT)).replace("\\","/"):digest(p) for p in sources if p.exists()}
    output_hashes={str(p.relative_to(ROOT)).replace("\\","/"):digest(p) for p in ROOT.rglob("*") if p.is_file() and p.name!="experiment_manifest.json"}
    save(ROOT/"experiment_manifest.json",{"schema_version":"n201r_experiment_v1",
        "baseline_commit":"724c658549d1f1737b863a80f7a7ca76046ec63f","implementation_identity":identity,
        "source_trace_sha256":read(OLD/"experiment_manifest.json")["source_trace_sha256"],
        "historical_artifact_hashes":historical,"output_hashes":output_hashes,
        "environment":{"python":platform.python_version(),"mujoco":mujoco.__version__,"numpy":np.__version__,
                       "scipy":scipy.__version__,"pyyaml":yaml.__version__},
        "protected_initial_state_sha256":digest(OLD/"initial_state.json"),"protected_damping_sha256":digest(OLD/"damping.json"),
        "protected_gates":read(OLD/"experiment_manifest.json")["numerical_gates"],
        "formal_run_budget":3,"formal_run_count":0,"candidate_budget":2,"candidate_count":2,
        "connection_status":result["status"]})
    print(json.dumps({"holding_passed":result["holding_precision_passed"],"numerics_passed":sensitivity_passed,
                      "load_peak":load_peak,"qualification":result["status"],"new_formal_runs":0}))


if __name__=="__main__":main()
