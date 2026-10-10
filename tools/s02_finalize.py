"""Finalize derived handoff identities after the read-only completion audit."""
import re
import time
from v6_mujoco.system_capture.estimation.common import OUT,PROJECT,read,save,sha,digest
from v6_mujoco.system_capture.estimation.runner import verify_frozen,resource_used

def main():
    verify_frozen()
    assert read(OUT/'completion_audit.json')['passed']
    book=read(OUT/'run_ledger.json')
    if not any(x['kind']=='uninstrumented_completion_reserve' for x in book['operations']):
        book['operations'].append(dict(kind='uninstrumented_completion_reserve',cpu_s=None,
            budget_charge_s=120.+60.*len(book['attempts']),actual_cpu_s='NOT_RECORDED',
            basis='conservative reservation: 60 s per attempted-run derived summary, plus 120 s for imports, manual reporting/figure iterations and final version-control/hashing overhead; not an exact measurement',
            new_robot_steps=0))
        save(OUT/'run_ledger.json',book)
    entries=[x for kind in ('operations','attempts','replays') for x in book[kind]]
    measured=sum(x.get('cpu_s') or 0. for x in entries)
    charged=resource_used()
    accounting=dict(measured_instrumented_cpu_s=measured,total_budget_charge_s=charged,
        explicit_nonmeasured_reservations_s=charged-measured,cpu_budget_s=21600,within_budget=charged<21600,
        elapsed_wall_s=time.time()-book['started_epoch'],
        operation_wall_s_sum=sum(x.get('wall_s') or 0. for x in entries),
        scope='measured timer scopes plus separately named conservative reservations; operation wall times may overlap and are not summed elapsed time',
        formal_robot_attempts=len(book['attempts']),physical_steps=sum(x['physical_steps'] for x in book['attempts']),
        online_predictive_steps=sum(x['predictive_steps'] for x in book['attempts']),
        replay_physical_steps=sum(x.get('physics_steps',0) for x in book['replays']),
        replay_predictive_steps=sum(x.get('predictive_steps',0) for x in book['replays']))
    assert accounting['within_budget']
    save(OUT/'resource_accounting.json',accounting)
    report=OUT/'report.md'
    text=report.read_text(encoding='utf-8')
    text=re.sub(r'当前计入预算 .*?未认证硬实时。',
        f"实测计时范围内CPU合计 {measured:.3f} s；加上明确列出的非精确保守预留后计入预算 {charged:.3f} s / 21600 s。物理步数 {accounting['physical_steps']}，在线预测步数 {accounting['online_predictive_steps']}。逐项CPU、墙钟与计费范围见 resource_accounting.json 和 run_ledger.json。每次更新/预测耗时见 estimator_timing.json 和逐运行 timing.json；写盘与序列化开销见 persistence_performance.json。未认证硬实时。",text)
    report.write_text(text,encoding='utf-8')
    h=read(OUT/'handoff.json')
    evidence={p.relative_to(PROJECT).as_posix():sha(p) for p in OUT.rglob('*') if p.is_file()
        and p.name not in ('handoff.json','operation.lock') and not p.name.endswith(('.tmp','.pyc')) and '__pycache__' not in p.parts}
    paths=list((PROJECT/'v6_mujoco/system_capture/estimation').glob('*.py'))
    paths+=list((PROJECT/'tools').glob('s02_*.py'))
    paths+=[PROJECT/p for p in ('v6_mujoco/system_capture/cli.py','v6_mujoco/system_capture/registry.py',
        'configs/system_capture/algorithms/s02_acceleration.json','configs/system_capture/protocols/s02.json',
        'experiments/system_capture/manifests/CODEX_S02_ESTIMATION_CAPTURE_AFTER_S01.md',
        'paper/system_framework/s02_estimation_section.md','.gitattributes')]
    evidence.update({p.relative_to(PROJECT).as_posix():sha(p) for p in paths})
    h.update(evidence_sha256=evidence,evidence_set_sha256=digest(evidence),
        completion_audit='completion_audit.json',resource_accounting=accounting,
        qualification=read(OUT/'qualification.json'),
        next_stage_policy='S03 requires a separate instruction; ideal and nonideal admission are separate in qualification')
    save(OUT/'handoff.json',h)
    errors=[rel for rel,expected in h['evidence_sha256'].items() if sha(PROJECT/rel)!=expected]
    assert not errors,errors
    print(dict(handoff_hashes_verified=len(evidence),budget_charge_s=charged,measured_cpu_s=measured))

if __name__=='__main__':main()
