"""S02 identities, audit and accounting. Historical outputs are read only."""
import time
from pathlib import Path
from ..common import PROJECT, OLD, read, save, sha, digest, git, environment

OUT = PROJECT / 'output/fpmfc/system_capture/S02'
BASE = '71caa4149176f9becd76f95020cb43e97dabc851'
BRANCH = 'codex/system-s02-estimation-capture-contract'
RUNS = ('R01_V2_nominal', 'R04_V2_H1', 'R02_V2_H2', 'R03_V2_S01')
PROTOCOL = PROJECT / 'configs/system_capture/protocols/s02.json'

def charge(kind, cpu, wall, **details):
    path = OUT / 'run_ledger.json'
    book = read(path)
    book['operations'].append(dict(kind=kind, cpu_s=time.process_time()-cpu,
                                  wall_s=time.perf_counter()-wall, **details))
    save(path, book)

def identity():
    paths = list((PROJECT/'v6_mujoco').rglob('*.py'))
    paths += list((PROJECT/'configs').rglob('*.yaml'))
    paths += list((PROJECT/'configs/system_capture').rglob('*.json'))
    paths += list((PROJECT/'models').rglob('*.xml'))
    paths += list((PROJECT/'assets/meshes/rizon4s/collision').glob('*.stl'))
    return {p.relative_to(PROJECT).as_posix(): sha(p) for p in sorted(paths)}

def prepare():
    cpu, wall = time.process_time(), time.perf_counter()
    if git('branch', '--show-current') != BRANCH or git('merge-base', BASE, 'HEAD') != BASE:
        raise RuntimeError('S02 baseline/branch mismatch')
    OUT.mkdir(parents=True, exist_ok=True)
    if not (OUT/'run_ledger.json').exists():
        save(OUT/'run_ledger.json', dict(phase='S02', cpu_budget_s=21600, max_attempts=4,
             attempts=[], operations=[], replays=[], next_phases_started=[], started_epoch=time.time()))
    files = {}
    def record(p, expected=None):
        p=Path(p); actual=sha(p)
        if expected is not None and actual != expected:
            raise RuntimeError('archive identity mismatch: '+str(p))
        with p.open('rb') as f:
            if f.read(50).startswith(b'version https://git-lfs.github.com'):
                raise RuntimeError('required LFS data not materialized: '+str(p))
        files[p.relative_to(PROJECT).as_posix()]=actual
    for phase in ('S00','S01'):
        d=PROJECT/'output/fpmfc/system_capture'/phase
        for name in ('report.md','qualification.json','handoff.json'):
            record(d/name)
    h=read(PROJECT/'output/fpmfc/system_capture/S01/handoff.json')
    for name in ('report.md','qualification.json'):
        p='output/fpmfc/system_capture/S01/'+name
        record(PROJECT/p,h['evidence_sha256'].get(p))
    for name in ('phase_manifest.json','baseline_equivalence.json','source_traceability.csv','legacy_inventory.csv'):
        record(PROJECT/'output/fpmfc/system_capture/S00'/name)
    for p in ('paper/system_paper/theory_notes.md','paper/system_paper/manuscript_v1.md',
              'paper/system_paper/limitations.md','output/fpmfc/n209_paper_system/uncertainty_floor.json',
              'experiments/system_capture/manifests/CODEX_SYSTEM_FRAMEWORK_AND_STEPWISE_PLAN.md',
              'experiments/system_capture/manifests/CODEX_S02_ESTIMATION_CAPTURE_AFTER_S01.md'):
        record(PROJECT/p)
    archives={}
    for run in RUNS:
        d=OLD/'runs'/run; cfg=read(d/'config.json'); v=read(d/'validation_timestamp_audited.json')
        if not(v['actuator_replay_passed'] and v['decision_replay_passed']):
            raise RuntimeError('historical dual validation missing: '+run)
        for name in ('config.json','metrics.json','validation_timestamp_audited.json','packets.jsonl.gz',
                     'trace.npz','control_states.json','estimator_events.json','progress_governor.json'):
            expected=v.get('trace_sha256') if name=='trace.npz' else v.get('original_control_states_sha256') if name=='control_states.json' else None
            record(d/name,expected)
        for rel, expected in cfg['identity'].items():
            record(OLD/'source_versions'/cfg['source_key']/rel if rel.endswith('.py') else PROJECT/rel, expected)
        archives[run]=dict(source_key=cfg['source_key'], validation='PASS_TIMESTAMP_AUDITED',
            task_status=read(d/'metrics.json')['status'], trace_sha256=v['trace_sha256'])
    save(OUT/'entry_acceptance.json',dict(status='PASS_WITH_EXPLICIT_HISTORICAL_EXCEPTION',
        baseline=BASE,branch=BRANCH,files_sha256=files,archives=archives,environment=environment(),
        S01_status='PARTIAL', new_robot_attempts=0, audit_epoch=time.time()))
    auth=PROJECT/'experiments/system_capture/manifests/CODEX_S02_ESTIMATION_CAPTURE_AFTER_S01.md'
    save(OUT/'legacy_gap_acceptance.json',dict(status='MISSING_ORIGINAL_TRACE',
        authorization_source='User /goal attachment explicitly authorizes S02 and limited C1 exception',
        authorization_file=auth.relative_to(PROJECT).as_posix(),authorization_sha256=sha(auth),
        recorded_at_epoch=time.time(), authorization_timestamp='NOT_SEPARATELY_ASSERTED',
        scope='C1 gap does not block independent S02; no retroactive pass, calibration use or fabricated trace',
        S01_status_retained='PARTIAL', signed_approval='NOT_FABRICATED'))
    charge('entry_identity_audit',cpu,wall,files=len(files),new_physics_steps=0)
    return dict(status='PREPARED',files=len(files))
