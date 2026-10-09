"""Immutable evidence contract and atomic accounting."""
import json
import platform
from functools import lru_cache
import time
from pathlib import Path
import numpy as np
import mujoco
import yaml
from v6_mujoco.model import PROJECT_ROOT
from v6_mujoco.postgrasp.physics import digest
from v6_mujoco.postgrasp_campaign.io import read,save,save_npz,identity,check_identity

ROOT=PROJECT_ROOT/"output/fpmfc/n205_end_to_end"
PREV=PROJECT_ROOT/"output/fpmfc/postgrasp_campaign"
C1=PROJECT_ROOT/"output/fpmfc/n110/live_twist_handoff"
MODEL=PROJECT_ROOT/"models/flexiv_rizon4s_end_to_end_scene.xml"
CONFIG=PROJECT_ROOT/"configs/n205_end_to_end_capture.yaml"
PHASE={"APPROACH":0,"CAPTURE_CHECK":1,"LATCH":2,"GRASP_VERIFY":3,"DAMP_TRANSFER":4,"HOLD":5,"FAILED":6}

@lru_cache(None)
def config(): return yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
@lru_cache(None)
def preconfig(): return yaml.safe_load((PROJECT_ROOT/"configs/fpmfc_paper_planning45_effective.yaml").read_text(encoding="utf-8"))
@lru_cache(None)
def contactconfig(): return yaml.safe_load((PROJECT_ROOT/"configs/fpmfc_contact.yaml").read_text(encoding="utf-8"))
@lru_cache(None)
def legacy(): return read(PREV/"campaign_manifest.json")
def runtime_identity(): return identity(sorted(Path(__file__).parent.glob("*.py")))
def arrays(path):
    with np.load(path,allow_pickle=False) as z:return {k:z[k] for k in z.files}
def ledger_add(kind,item):
    p=ROOT/"run_ledger.json"; ledger=read(p);ledger[kind].append(item);save(p,ledger)
def verify_design():
    m=read(ROOT/"experiment_manifest.json");check_identity(m["source_identity"]);check_identity(m["design_identity"]);return m

def prepare():
    if (ROOT/"experiment_manifest.json").exists(): return verify_design()
    check_identity(read(PREV/"source_identity.json")["files"])
    check_identity(read(PREV/"selected_controller.json")["evidence_identity"])
    pairing=read(C1/"pairing_manifest.json")
    assert pairing["source_candidate"]["capture_time_s"]==8
    metrics=read(C1/"precontact/C1/metrics.json")
    assert read(C1/"precontact/C1/validation.json")["replay_passed"]
    assert digest(C1/"precontact/C1/trace.npz")==metrics["trace_sha256"]
    assert (C1/"precontact/C1/trace.npz").read_bytes()[:2]==b"PK"
    from .adapter import contract
    contracts=contract()
    for name,value in contracts.items():save(ROOT/(name+".json"),value)
    sources=[p for p in (PROJECT_ROOT/"v6_mujoco").rglob("*.py") if "end_to_end_capture" not in p.parts]
    sources+=list((PROJECT_ROOT/"assets/meshes/rizon4s/collision").glob("*.stl"))
    sources += [p for p in (PROJECT_ROOT/"configs").glob("*.yaml") if p!=CONFIG]
    sources += list((PROJECT_ROOT/"models").glob("*.xml"))
    sources=[p for p in sources if p!=MODEL]
    sources += [PREV/"campaign_manifest.json",PREV/"selected_controller.json",PREV/"source_identity.json",PREV/"qualification_matrix.json",PROJECT_ROOT/"paper/POSTGRASP_DETUMBLING_CAMPAIGN_REPORT.md",PROJECT_ROOT/"output/fpmfc/n200_postgrasp/damping.json",C1/"pairing_manifest.json",C1/"precontact/C1/trace.npz",C1/"precontact/C1/metrics.json",C1/"precontact/C1/validation.json"]
    manifest={"schema":"n205_v1","frozen_epoch":time.time(),"baseline":"7c521550c39e7cea97c83634e21504c888f5f8af","design":config(),"source_identity":identity(sources),"design_identity":identity([CONFIG,MODEL]+[ROOT/(n+".json") for n in contracts]),"legacy_gates":legacy()["legacy_gates"],"capture_gates":legacy()["design"]["latch"],"replay_tolerances":legacy()["design"]["replay_tolerances"],"fine_gates":legacy()["design"]["step_sensitivity_gates"],"environment":{"python":platform.python_version(),"mujoco":mujoco.__version__,"numpy":np.__version__},"legacy_full_boundary_contract":"FAILED","restricted_fixture_domain":"inherited_verified_scope","initialization":"home robot plus target.sample(0), named addresses; no terminal injection","observation":"all forward/finite-difference work isolated from real MjData","runtime_identity_policy":"freeze each attempt before first physics step; preserve failed identities and traces","optional_Z":"NOT_RUN; omitted to keep this integration campaign focused","media":"numerically replayed nominal overview, interface closeup, five-view composite and body-side; partial trace labelled with real duration on failure"}
    save(ROOT/"experiment_manifest.json",manifest)
    save(ROOT/"run_ledger.json",{"formal_runs":[],"replays":[],"unit_tests":[],"skipped":{},"implementation_failures":[]})
    return manifest
