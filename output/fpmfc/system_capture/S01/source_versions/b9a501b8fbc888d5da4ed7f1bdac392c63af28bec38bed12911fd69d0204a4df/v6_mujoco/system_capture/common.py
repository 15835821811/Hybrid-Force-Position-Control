"""S00 paths, content identities and atomic JSON, independent of legacy ledgers."""
import hashlib
import json
import os
import platform
import subprocess
import sys
from pathlib import Path
from .contracts import plain

PROJECT=Path(os.environ.get('SYSTEM_CAPTURE_PROJECT_ROOT',Path(__file__).resolve().parents[2]))
OUT=PROJECT/'output/fpmfc/system_capture/S00'
REGISTRY=PROJECT/'experiments/system_capture/registry'
BASE='007abc48fe3d804a96d4feee0a1c304444048669'
OLD=PROJECT/'output/fpmfc/n209_paper_system'
RUNS=('R01_V2_nominal','R03_V2_S01')

def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''): h.update(block)
    return h.hexdigest()

def digest(x):
    return hashlib.sha256(json.dumps(plain(x),sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()

def read(p): return json.loads(Path(p).read_text(encoding='utf-8'))

def save(p,x):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
    temp=p.with_suffix(p.suffix+'.tmp')
    temp.write_text(json.dumps(plain(x),ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    os.replace(temp,p)

def git(*args):
    return subprocess.check_output(['git','-c','core.longpaths=true',*args],cwd=PROJECT).decode('utf-8').strip()

def source_identity():
    paths=list((PROJECT/'v6_mujoco/system_capture').rglob('*.py'))
    paths+=list((PROJECT/'configs/system_capture').rglob('*.json'))
    return {p.relative_to(PROJECT).as_posix():sha(p) for p in sorted(paths)}

def environment():
    import mujoco,numpy,scipy,yaml
    return {'python':platform.python_version(),'executable':sys.executable,'mujoco':mujoco.__version__,
            'numpy':numpy.__version__,'scipy':scipy.__version__,'pyyaml':yaml.__version__,
            'platform':platform.platform(),'machine':platform.machine(),'processor':platform.processor(),
            'logical_cpu_count':os.cpu_count(),'threads':{k:os.environ.get(k) for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS')}}

def clean(x):
    import numpy as np
    x=plain(x)
    if isinstance(x,dict):return {k:clean(v) for k,v in x.items()}
    if isinstance(x,list):return [clean(v) for v in x]
    if isinstance(x,float) and not np.isfinite(x):return None
    return x
