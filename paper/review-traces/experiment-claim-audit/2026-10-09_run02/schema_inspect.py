from audit_inspect import *
import numpy as np
ROOT=PAPER.parent
base=ROOT/'output/fpmfc/n209_paper_system'
def shape(x, depth=0):
    if isinstance(x,dict): return {k:shape(v,depth+1) for k,v in x.items()} if depth<2 else list(x.keys())
    if isinstance(x,list): return {'length':len(x),'first':shape(x[0],depth+1) if x else None}
    return x
for f in ['runs/D03_V1_S01/metrics.json','runs/D03_V1_S01/config.json','runs/D03_V1_S01/events.json','runs/D03_V1_S01/validation_timestamp_audited.json','runs/D03_V1_S01/estimator_events.json','runs/D03_V1_S01/control_states.json','runs/D03_V1_S01/posteriors.json','runs/D03_V1_S01/tasks.json','runs/D03_V1_S01/progress_governor.json','diagnostics/H2_prior/snapshots.json','diagnostics/S01_noise_delay/new_estimator_redecision.json']:
    d=read(base/f)
    print('\nFILE',f,'\n',json.dumps(shape(d),ensure_ascii=False))
for p in [base/'runs/D03_V1_S01/trace.npz',ROOT/'output/fpmfc/n208_adaptive_capture/runs/S01_noise_delay/trace.npz',ROOT/'output/fpmfc/n208_adaptive_capture/runs/H1_prior/regression_blocks.npz']:
    assert p in FILES.values()
    with np.load(p,allow_pickle=False) as z:
        print('\nNPZ',p.name,[(k,z[k].shape,str(z[k].dtype)) for k in z.files])
for f in ['identification.json','governor.json','posteriors.json']:
    print('\nH1',f,json.dumps(shape(read(ROOT/'output/fpmfc/n208_adaptive_capture/runs/H1_prior'/f)),ensure_ascii=False))
