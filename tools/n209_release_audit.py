"""Read-only release check; no robot trajectory, retuning or changed verdicts."""
import ast
import csv
import hashlib
import json
import re
import sys
from pathlib import Path, PureWindowsPath

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from v6_mujoco.feasible_capture.common import ROOT,PAPER,PROJECT_ROOT,read,save,sha

required=['parameter_policy.md','baseline_contract.json','qp_failure_diagnosis.json','sensor_reference_audit.json','development_ledger.json',
          'frozen_manifest.json','holdout_generator_contract.json','holdout_identity.json','comparison.json','ablation_summary.json',
          'identifiability_supporting_audit.json','statistics.json','qualification_matrix.json','run_ledger.json','source_manifest.json','commands.md','tests.log',
          'uncertainty_floor.json','logging_integrity_audit.json','resource_summary.json','git_evidence_check.json']
required_paper=['manuscript_v1.md','related_work_matrix.csv','references.bib','notation_and_assumptions.md','theory_notes.md',
                'claim_evidence_matrix.csv','experiment_protocol.md','limitations.md','submission_gap_report.md','figure_review.md']
errors=[]
for base,names in [(ROOT,required),(PAPER,required_paper)]:
    for name in names:
        if not (base/name).is_file() or not (base/name).stat().st_size:errors.append('missing '+str(base/name))
for p in (PROJECT_ROOT/'v6_mujoco/feasible_capture').glob('*.py'):
    try:ast.parse(p.read_text(encoding='utf-8'))
    except SyntaxError as ex:errors.append(str(ex))
manifest=read(ROOT/'source_manifest.json')
for rel,item in manifest['sources'].items():
    if rel.startswith('output/fpmfc/n208_') or rel=='paper/N208_PASSIVE_TARGET_ESTIMATION_CAPTURE_REPORT.md':
        if sha(PROJECT_ROOT/rel)!=item['sha256']:errors.append('historical source changed '+rel)
q=read(ROOT/'qualification_matrix.json');s=read(ROOT/'statistics.json');book=read(ROOT/'run_ledger.json')
assert not book['frozen'] and not q['performance_promotion']
if not q['engineering_delivery_complete']:errors.append('delivery validation incomplete')
if len(book['attempts'])>60:errors.append('total attempt budget')
for kind,limit in book['limits'].items():
    if sum(x['category']==kind for x in book['attempts'])>limit:errors.append('category budget '+kind)
for x in book['attempts']:
    p=ROOT/'runs'/x['name'];cfg=read(p/'config.json');m=read(p/'metrics.json');v=read(p/'validation_timestamp_audited.json')
    if m['trace_sha256']!=sha(p/'trace.npz') or v['trace_sha256']!=m['trace_sha256']:errors.append('trace identity '+x['name'])
    if not(v['actuator_replay_passed'] and v['decision_replay_passed']):errors.append('replay failed '+x['name'])
    if not m['full_window_evaluated'] and m['world_window_max_deg_s'] is not None:errors.append('failed window counted '+x['name'])
    archive=ROOT/'source_versions'/cfg['source_key']
    for rel,digest in cfg['identity'].items():
        if rel.endswith('.py') and sha(archive/rel)!=digest:errors.append('archive mismatch '+rel)
        if not rel.startswith('v6_mujoco/feasible_capture/') and rel not in ['v6_mujoco/adaptive_capture/controller.py','configs/n209_paper_system.yaml']:
            if sha(PROJECT_ROOT/rel)!=digest:errors.append('shared dependency differs '+rel)
    if cfg['mission']['parameter_feedback']:errors.append('parameter feedback enabled')
    if cfg['mission']['capture']!={'translation_m':.0001,'rotation_deg':.05,'linear_m_s':.001,'angular_deg_s':.2}:errors.append('capture gate changed')
    if cfg['mission']['post_duration_s']!=20 or cfg['mission']['approach_deadline_s']!=20:errors.append('mission budget changed')
bib=(PAPER/'references.bib').read_text(encoding='utf-8');keys=re.findall(r'@\w+\{([^,]+),',bib)
paper=(PAPER/'manuscript_v1.md').read_text(encoding='utf-8');cited=set(re.findall(r'@([\w]+)',paper))
if len(keys)!=8 or len(set(keys))!=8 or not cited<=set(keys):errors.append('citation keys mismatch')
matrix=list(csv.DictReader((PAPER/'related_work_matrix.csv').open(encoding='utf-8')))
if set(x['key'] for x in matrix)!=set(keys):errors.append('literature matrix mismatch')
for target in re.findall(r'!?\[[^\]]*\]\(([^)]+)\)',paper):
    if not target.startswith(('https://','http://','#')) and not (PAPER/target).exists():errors.append('broken manuscript link '+target)
if '{{' in paper:errors.append('unexpanded paper template')
audit=read(PAPER.parent/'PAPER_CLAIM_AUDIT.json')
if audit['verdict'] not in ['PASS','WARN']:errors.append('paper claim audit not finalized')
declared=read(PAPER.parent/'review-traces/experiment-claim-audit/2026-10-09_run02/inputs.json')['declared_input_hashes']
if audit['audited_input_hashes']!=declared:errors.append('claim audit differs from exact declared input set')
for key,digest in audit['audited_input_hashes'].items():
    p=Path(key) if PureWindowsPath(key).is_absolute() else PAPER.parent/key
    if not p.is_file() or 'sha256:'+sha(p)!=digest:errors.append('stale claim audit input '+key)
if audit['audited_input_hashes'].get('system_paper/manuscript_v1.md')!='sha256:'+sha(PAPER/'manuscript_v1.md'):
    errors.append('final manuscript not in declared audit set')
floor=read(ROOT/'uncertainty_floor.json')
if not(floor['minimum_velocity_3sigma_lower_bound_m_s']>floor['capture_linear_limit_m_s'] and floor['riccati_fixed_point_residual']<1e-15):errors.append('covariance floor certificate')
integrity=read(ROOT/'logging_integrity_audit.json')
if {r['name'] for r in integrity['runs'] if r['differences']}!={'D03_V1_S01','R03_V2_S01'}:errors.append('unexpected logging differences')
if read(ROOT/'runs/D03_V1_S01/validation.json')['decision_replay_passed']:errors.append('original failed validation missing')
resource=read(ROOT/'resource_summary.json')
if resource['measured_process_cpu_s']>28800 or resource['elapsed_campaign_wall_s']>28800:errors.append('resource budget exceeded')
if not read(ROOT/'git_evidence_check.json')['passed']:errors.append('Git/LFS evidence check failed')
claims=list(csv.DictReader((PAPER/'claim_evidence_matrix.csv').open(encoding='utf-8')))
for row in claims:
    if '*' not in row['file'] and not (PROJECT_ROOT/row['file']).exists():errors.append('broken claim file '+row['claim_id'])
result={'passed':not errors,'errors':errors,'required_artifacts':len(required)+len(required_paper),'attempts':len(book['attempts']),
        'citation_entries':len(keys),'paper_claim_audit_status':audit['verdict'],'claim_audit_input_hashes_current':not any('claim audit' in e for e in errors),
        'scope':'artifact/identity/definition consistency; not new performance, safety proof, or independent scientific peer review',
        'validator_sha256':sha(Path(__file__)),'qualification_sha256':sha(ROOT/'qualification_matrix.json')}
save(ROOT/'release_audit.json',result);print(json.dumps(result,ensure_ascii=False));raise SystemExit(bool(errors))
