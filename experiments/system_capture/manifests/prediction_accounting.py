"""Read-only supplement: distinguish measured approach and derived brake steps."""
import json
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from v6_mujoco.system_capture.common import PROJECT, OLD, OUT, RUNS, read, save, sha

rows = {}
for name in RUNS:
    result = read(OUT / 'replays' / (name + '__decision.json'))
    assert result['passed'] and result['intermediate_match'] and result['final_outcome_matches']
    folder = OLD / 'runs' / name
    approach = read(folder / 'progress_governor.json')
    brake = read(folder / 'governor.json')
    candidates = [c for call in approach for c in call['candidates']]
    predictions = [p for call in brake for p in call['predictions']]
    assert all(p['safe_in_sampled_models'] for p in predictions), 'Rejected brake prediction requires separate loop-step evidence'
    archived = OLD / 'source_versions' / result['source_key'] / 'v6_mujoco/adaptive_capture/governor.py'
    source = archived.read_text(encoding='utf-8')
    assert 'for _ in range(3)' in source and 'for step in range(11)' in source and 'if step<10:mujoco.mj_step(m,d)' in source
    measured = result['predictive_steps']
    assert sum(c['steps'] for c in candidates) == measured
    derived = len(predictions) * 3 * 10
    rows[name] = {
        'approach_steps_measured_in_replay': measured,
        'approach_calls': len(approach), 'approach_candidates': len(candidates),
        'approach_rejected_candidates': sum(not c['verified'] for c in candidates),
        'brake_calls_in_archived_log': len(brake),
        'brake_predictions_in_archived_log': len(predictions),
        'brake_rejected_predictions': 0,
        'brake_steps_derived_from_verified_log_and_unchanged_loop': derived,
        'combined_measured_plus_derived_steps': measured + derived,
        'source_sha256': sha(archived),
        'log_hashes': {p.name: sha(p) for p in [folder / 'governor.json', folder / 'progress_governor.json']},
    }
save(OUT / 'prediction_accounting.json', {
    'scope': 'Supplement after replay; no controller execution or source mutation. metrics.decision_predictive_steps is the measured approach-governor counter only. CPU includes both approach and brake predictions.',
    'brake_derivation': 'Each retained safe brake candidate completes 3 independent models x 10 mj_step calls. Derived from original logs and exact archived source, conditional on verified decision/state equality; not directly instrumented in the replay.',
    'per_run': rows, 'combined_steps': sum(r['combined_measured_plus_derived_steps'] for r in rows.values()),
    'supplement_source_sha256': sha(Path(__file__)), 'new_physical_attempts': 0, 'additional_replays': 0,
})
print(json.dumps(rows))
