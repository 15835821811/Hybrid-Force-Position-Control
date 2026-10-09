"""Check staged N209 bytes and local LFS objects before the local commit."""
import hashlib
import json
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from v6_mujoco.feasible_capture.common import ROOT, PROJECT_ROOT, save, sha

def git(*args, **kwargs):
    return subprocess.check_output(['git', '-c', 'core.longpaths=true', *args], cwd=PROJECT_ROOT, **kwargs)

prefixes = ('output/fpmfc/n209_paper_system/', 'paper/system_paper/', 'paper/review-traces/',
            'v6_mujoco/feasible_capture/', 'tools/n209_', 'paper/PAPER_CLAIM_AUDIT.')
singles = {'configs/n209_paper_system.yaml', 'v6_mujoco/adaptive_capture/controller.py',
           'paper/N209_FEASIBILITY_AND_UNCERTAINTY_REPORT.md'}
entries = []
for row in git('ls-files', '--stage', '-z').split(b'\0'):
    if not row:
        continue
    meta, raw_path = row.split(b'\t', 1)
    path = raw_path.decode('utf-8')
    if path.endswith('/git_evidence_check.json'):
        continue
    if path.startswith(prefixes) or path in singles:
        entries.append((path, meta.split()[1].decode()))
oids = list(dict.fromkeys(oid for _, oid in entries))
raw = git('cat-file', '--batch', input=('\n'.join(oids)+'\n').encode())
objects = {}
offset = 0
for oid in oids:
    end = raw.index(b'\n', offset)
    header = raw[offset:end].split()
    size = int(header[2]); start = end + 1
    objects[oid] = raw[start:start+size]
    offset = start + size + 1
common = Path(git('rev-parse', '--git-common-dir').decode().strip())
if not common.is_absolute():
    common = PROJECT_ROOT / common
errors = []; lfs = set(); checked = 0
for path, oid in entries:
    data = objects[oid]
    if data.startswith(b'version https://git-lfs.github.com/spec/v1\n'):
        fields = dict(line.split(' ', 1) for line in data.decode().splitlines()[1:])
        digest = fields['oid'].removeprefix('sha256:')
        if sha(PROJECT_ROOT/path) != digest or (PROJECT_ROOT/path).stat().st_size != int(fields['size']):
            errors.append('working LFS bytes: '+path)
        if digest not in lfs:
            obj = common/'lfs/objects'/digest[:2]/digest[2:4]/digest
            if not obj.is_file() or sha(obj) != digest:
                errors.append('missing/corrupt local LFS object: '+digest)
        lfs.add(digest)
    elif sha(PROJECT_ROOT/path) != hashlib.sha256(data).hexdigest():
        errors.append('Git byte normalization or stale index: '+path)
    checked += 1
result = {'passed': not errors, 'errors': errors, 'staged_files_checked': checked,
          'unique_local_lfs_objects_verified': len(lfs), 'checker_sha256': sha(Path(__file__)),
          'scope': 'current staged N209 source/evidence bytes and local LFS availability; excludes this self-referential report'}
save(ROOT/'git_evidence_check.json', result)
print(json.dumps(result)); raise SystemExit(bool(errors))
