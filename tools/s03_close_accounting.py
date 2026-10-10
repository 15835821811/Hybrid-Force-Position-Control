"""Close deferred read-only process receipts without changing trial records."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from v6_mujoco.system_capture.planning.contracts import OUT,read,save,sha

def main():
    if (OUT/'operation.lock').exists():raise RuntimeError('close accounting only after robot/replay operation finishes')
    b=read(OUT/'candidate_budget.json');p=OUT/'runs/B01/decision_replay.json'
    if p.exists() and not b.get('B01_decision_replay_accounted'):
        r=read(p);assert r['passed']
        b['replay_predictive_steps']+=r['predictive_steps'];b['B01_decision_replay_accounted']=True
        b['B01_replay_scope']='unchanged scalar predictor is not instrumented by S03 Budget; exact reported steps added once, no latch/brake calls'
        save(OUT/'candidate_budget.json',b)
    book=read(OUT/'run_ledger.json')
    for name in ['evidence_operation_receipt.json','report_preview_receipt.json','scientific_plots_receipt.json']:
        p=OUT/name
        if p.exists() and not any(x.get('receipt')==name for x in book['operations']):
            book['operations'].append(dict(**read(p),receipt=name,receipt_sha256=sha(p)))
    if not any(x['kind']=='support_process_reserve' for x in book['operations']):
        book['operations'].append(dict(kind='support_process_reserve',cpu_s=None,budget_charge_s=600,exact=False,
            scope='conservative CPU allowance for unmetered shell/bootstrap/source recovery, post-run summary work and failed small analysis commands; separate from initial120s inspection allowance'))
    save(OUT/'run_ledger.json',book)
    print(dict(candidate_evaluations=b['evaluations'],predictive_steps=b['predictive_steps']+b['replay_predictive_steps'],actual_attempts=len(book['attempts'])))

if __name__=='__main__':main()
