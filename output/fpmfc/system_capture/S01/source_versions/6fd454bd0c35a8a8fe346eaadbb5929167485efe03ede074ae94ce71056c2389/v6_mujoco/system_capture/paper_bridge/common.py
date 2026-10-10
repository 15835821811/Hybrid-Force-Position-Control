import hashlib
import json
import os
import subprocess
import sys
import time
from functools import lru_cache
from pathlib import Path
import numpy as np
import yaml
from ..contracts import plain

PROJECT=Path(__file__).resolve().parents[3]
OUT=PROJECT/'output/fpmfc/system_capture/S01'
CONFIG=PROJECT/'configs/system_capture/paper_compat'
PROTOCOL=PROJECT/'configs/system_capture/protocols/s01.json'
BASE='382d3b36287f84f68e6a458331c45d895fd9fe40'
def read(p):return json.loads(Path(p).read_text(encoding='utf-8'))
def yaml_read(p):return yaml.safe_load(Path(p).read_text(encoding='utf-8'))
@lru_cache(None)
def source():return yaml_read(CONFIG/'source_parameters.yaml')
@lru_cache(None)
def assumptions():return yaml_read(CONFIG/'assumptions.yaml')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def digest(obj):return hashlib.sha256(json.dumps(plain(obj),sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
def save(p,obj):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
    t=p.with_suffix(p.suffix+'.tmp');t.write_text(json.dumps(plain(obj),ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');os.replace(t,p)
def git(*args):return subprocess.check_output(['git','-c','core.longpaths=true',*args],cwd=PROJECT).decode('utf-8').strip()
def wrap(x):return (np.asarray(x)+np.pi)%(2*np.pi)-np.pi
def skew(x):
    a,b,c=x;return np.array([[0.,-c,b],[c,0.,-a],[-b,a,0.]])
