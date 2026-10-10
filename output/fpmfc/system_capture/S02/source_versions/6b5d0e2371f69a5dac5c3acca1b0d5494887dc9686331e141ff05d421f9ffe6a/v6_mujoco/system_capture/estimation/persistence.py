"""Append-only committed chunks; proposals, applications and integration differ.

A block becomes committed only when its hash appears in the atomic index.
Unindexed blocks/tails are ignored, never guessed or repaired during replay.
fsync is used, but no universal storage/power-loss guarantee is made.
"""
import gzip
import json
import os
import time
from pathlib import Path
from ..contracts import plain
from ..common import sha, read

def atomic_bytes(path,data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_suffix(path.suffix+'.tmp')
    with temp.open('wb') as f:
        f.write(data);f.flush();os.fsync(f.fileno())
    os.replace(temp,path)

def atomic_json(path,value):
    atomic_bytes(path,(json.dumps(plain(value),ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n').encode())

class Journal:
    def __init__(self,directory,block_records=150):
        self.path=Path(directory);self.path.mkdir(parents=True,exist_ok=False)
        self.block_records=block_records;self.buffer=[];self.next_index=0;self.io_s=0.;self.serialization_s=0.
        self.index=dict(schema='S02_journal_v1',blocks=[],committed_records=0,max_completed_time_s=0.,
            tail_policy='only indexed checksum-verified blocks are replayable; uncommitted tail may be missing')
        atomic_json(self.path/'index.json',self.index)

    def append(self,event):
        start=time.perf_counter()
        event=dict(sequence=self.next_index,**event)
        # Serialize on append: immutable point-in-time bytes, never mutable alias.
        data=json.dumps(plain(event),ensure_ascii=False,separators=(',',':'),allow_nan=False).encode()+b'\n'
        self.buffer.append(data);self.next_index+=1;self.serialization_s+=time.perf_counter()-start
        if len(self.buffer)>=self.block_records:self.commit()

    def commit(self):
        if not self.buffer:return
        start=time.perf_counter();number=len(self.index['blocks']);name=f'block_{number:06d}.jsonl.gz'
        raw=b''.join(self.buffer);data=gzip.compress(raw,compresslevel=3,mtime=0)
        atomic_bytes(self.path/name,data)
        rows=[json.loads(x) for x in self.buffer]
        block=dict(file=name,sha256=sha(self.path/name),first=rows[0]['sequence'],last=rows[-1]['sequence'],records=len(rows))
        new=dict(self.index);new['blocks']=self.index['blocks']+[block];new['committed_records']=rows[-1]['sequence']+1
        completed=[r['time'] for r in rows if r['kind']=='step_completed']
        if completed:new['max_completed_time_s']=max(completed)
        atomic_json(self.path/'index.json',new)
        self.index=new;self.buffer.clear();self.io_s+=time.perf_counter()-start

    def close(self):
        self.commit()
        return validate(self.path)

def records(directory):
    path=Path(directory);index=read(path/'index.json');expected=0
    for b in index['blocks']:
        p=path/b['file']
        if sha(p)!=b['sha256']:raise RuntimeError('committed chunk checksum mismatch')
        with gzip.open(p,'rt',encoding='utf-8') as f:rows=[json.loads(line) for line in f]
        if len(rows)!=b['records'] or b['first']!=expected or b['last']!=expected+len(rows)-1:raise RuntimeError('chunk range mismatch')
        for r in rows:
            if r['sequence']!=expected:raise RuntimeError('non-contiguous record sequence')
            yield r;expected+=1
    if expected!=index['committed_records']:raise RuntimeError('index count mismatch')

def completed_inputs(directory):
    """Only match an actual application to a completed integrator step."""
    applied=None
    for r in records(directory):
        if r['kind']=='input_applied':
            if applied is not None:raise RuntimeError('application without completed previous step')
            applied=r
        elif r['kind']=='step_completed':
            if applied is None or r['step']!=applied['step']:raise RuntimeError('completion without matching input')
            yield applied,r;applied=None
    # A committed application with no completed integration is an incomplete
    # tail, not evidence of execution and not an invented step.

def validate(directory):
    count=0;last=0.
    for _,r in completed_inputs(directory):count+=1;last=r['time']
    idx=read(Path(directory)/'index.json')
    if abs(last-idx['max_completed_time_s'])>1e-12:raise RuntimeError('completed-time mismatch')
    return dict(passed=True,records=idx['committed_records'],completed_steps=count,max_verifiable_time_s=last,
        block_count=len(idx['blocks']),index_sha256=sha(Path(directory)/'index.json'))
