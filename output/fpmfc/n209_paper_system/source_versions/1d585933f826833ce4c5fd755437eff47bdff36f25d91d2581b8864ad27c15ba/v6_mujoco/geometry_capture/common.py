"""N206 independent result root and immutable final admission identity."""
import json
import time
from pathlib import Path
import mujoco
import numpy as np
from v6_mujoco.model import PROJECT_ROOT
from v6_mujoco.postgrasp.physics import digest
from v6_mujoco.postgrasp_campaign.io import read,save,save_npz,identity,check_identity
from v6_mujoco.end_to_end_capture.common import preconfig,contactconfig,legacy,C1,PHASE,arrays
from .audit import ROOT
from .design import config,NEW_MODEL as MODEL,CONFIG

def runtime_identity():return identity(sorted(Path(__file__).parent.glob('*.py')))
def ledger_add(kind,item):
    ledger=read(ROOT/'run_ledger.json');ledger[kind].append(item);save(ROOT/'run_ledger.json',ledger)
def verify_design():
    m=read(ROOT/'experiment_manifest.json');check_identity(m['source_identity']);check_identity(m['design_identity'])
    assert read(ROOT/'unit_tests.json')['passed'] and read(ROOT/'interface_tests/result.json')['passed']
    assert read(ROOT/'trajectory_screening.json')['selected'] is not None
    return m
def prepare():
    from v6_mujoco.end_to_end_capture.common import verify_design as old_verify
    old_verify()
    if (ROOT/'experiment_manifest.json').exists():return verify_design()
    assert read(ROOT/'unit_tests.json')['passed'] and read(ROOT/'interface_tests/result.json')['passed']
    assert read(ROOT/'trajectory_screening.json')['selected'] is not None
    source=read(PROJECT_ROOT/'output/fpmfc/n205_end_to_end/experiment_manifest.json')['source_identity']
    source.update(identity([p for p in (PROJECT_ROOT/'v6_mujoco/end_to_end_capture').glob('*.py')]))
    docs=[ROOT/n for n in ['geometry_audit.json','selected_design.json','changed_mass_inertia.json','terminal_mating_contract.json','trajectory_screening.json','unit_tests.json','interface_tests/result.json']]
    manifest={'schema':'n206_v1','source_commit':config()['source_commit'],'design':config(),'source_identity':source,
              'design_identity':identity([MODEL,CONFIG]+docs),'runtime_identity_at_admission':runtime_identity(),
              'frozen_epoch':time.time(),'environment':{'mujoco':mujoco.__version__,'numpy':np.__version__},
              'hardware_design_assumption_changed':True,'real_gripper_and_hardware_validated':False,'formal_run_limit':4}
    save(ROOT/'experiment_manifest.json',manifest);return manifest
