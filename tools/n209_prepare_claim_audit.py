"""Collect an explicit immutable raw-input set for the final fresh reviewer."""
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from v6_mujoco.feasible_capture.common import ROOT, OLD, PAPER, PROJECT_ROOT, read, save, sha

trace = PAPER.parent / 'review-traces/experiment-claim-audit/2026-10-09_run02'
trace.mkdir(parents=True, exist_ok=True)
manifest = trace / 'inputs.json'
if manifest.exists():
    raise RuntimeError('Final review input set already exists; do not replace an audit identity')
paths = [PAPER / 'manuscript_v1.md', PROJECT_ROOT / 'configs/n209_paper_system.yaml']
paths += sorted((PAPER / 'generated_tables').glob('*.md'))
paths.append(PAPER / 'figure_plan.md')
paths += sorted((PROJECT_ROOT / 'configs/adaptive_capture').glob('*.yaml'))
for run in sorted((ROOT / 'runs').iterdir()):
    if not run.is_dir():
        continue
    for name in ['config.json', 'metrics.json', 'trace.npz', 'progress_governor.json',
                 'control_states.json', 'packets.jsonl.gz', 'events.json', 'estimator_events.json',
                 'posteriors.json', 'governor.json', 'tasks.json', 'identification.json',
                 'validation_timestamp_audited.json']:
        paths.append(run / name)
paths.append(ROOT / 'runs/D03_V1_S01/validation.json')
paths += sorted((ROOT / 'diagnostics').glob('*/snapshots.json'))
paths.append(ROOT / 'diagnostics/S01_noise_delay/new_estimator_redecision.json')
for name in ['S01_noise_delay', 'H1_prior', 'H1_identified', 'H3_prior', 'H3_identified']:
    for filename in ['trace.npz', 'posteriors.json', 'governor.json', 'config.json', 'identification.json', 'regression_blocks.npz']:
        p = OLD / 'runs' / name / filename
        if p.exists():
            paths.append(p)
paths += [ROOT / 'uncertainty_floor.json', ROOT / 'run_ledger.json', sorted(ROOT.glob('tests_*.json'))[-1]]
paths += sorted((ROOT / 'figures').glob('f[1-5]_*.png'))
hashes = {}
for path in paths:
    if not path.is_file():
        raise FileNotFoundError(path)
    key = path.relative_to(PAPER.parent).as_posix() if path.is_relative_to(PAPER.parent) else str(path.resolve())
    hashes[key] = 'sha256:' + sha(path)
save(manifest, {'paper_dir': str(PAPER.parent), 'declared_input_hashes': hashes,
                'generated_at': datetime.now(timezone.utc).isoformat(),
                'scope': 'paper source, figures, raw numerical records/configurations; no executor report or previous review'})
(trace / 'manuscript_input.md').write_bytes((PAPER / 'manuscript_v1.md').read_bytes())
print(json.dumps({'input_files': len(hashes), 'manifest': str(manifest)}))
