from pathlib import Path
import hashlib
import time
import yaml
from v6_mujoco.model import PROJECT_ROOT
from v6_mujoco.postgrasp_campaign.io import read, save, save_npz, identity

ROOT = PROJECT_ROOT / 'output/fpmfc/n208_adaptive_capture'
CONFIG = PROJECT_ROOT / 'configs/adaptive_capture'
def config(name='mission'):
    return yaml.safe_load((CONFIG / (name+'.yaml')).read_text(encoding='utf-8'))
def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def runtime_identity():
    paths = list((PROJECT_ROOT/'v6_mujoco').rglob('*.py')) + list(CONFIG.glob('*.yaml'))
    paths += [PROJECT_ROOT/'models/flexiv_rizon4s_n206_tool_scene.xml', PROJECT_ROOT/'configs/n206_geometry_capture.yaml', PROJECT_ROOT/'configs/fpmfc_paper_planning45_effective.yaml']
    return identity(sorted(paths))
def init_ledger():
    path=ROOT/'run_ledger.json'
    if not path.exists():
        save(path, {'started_epoch':time.time(), 'cpu_s':0., 'attempts':[], 'isolated_tests':[], 'replays':[], 'changes':[], 'frozen':False})
    return read(path)
