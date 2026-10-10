"""Audit snapshot timing without changing original records or physical verdicts."""
import numpy as np
from .common import ROOT,read,save,sha,ledger

def run():
    rows=[]
    for entry in ledger()['attempts']:
        path=ROOT/'runs'/entry['name'];cfg=read(path/'config.json');states=read(path/'control_states.json')
        with np.load(path/'trace.npz') as z:P=z['estimate_covariance']
        differences=[]
        for s in states:
            e=s['estimate']
            if e is None:continue
            k=round(s['time']/cfg['dt']);delta=float(np.max(abs(np.array(e['covariance'])-P[k])))
            if delta>1e-14:
                assert s['time']==0 and not e['valid'] and e['measurement_time']==-1.,'additional unknown logging mismatch'
                differences.append({'time':0.,'field':'estimate.covariance','difference':delta,'saved_diagonal':np.diag(e['covariance']),
                                    'correct_same_tick_trace_diagonal':np.diag(P[k]),'state_valid':False})
        rows.append({'name':entry['name'],'differences':differences,'control_states_sha256':sha(path/'control_states.json'),'trace_sha256':sha(path/'trace.npz')})
    result={'status':'KNOWN_T0_INVALID_ESTIMATE_SNAPSHOT_ALIAS','runs':rows,'affected_runs':[x['name'] for x in rows if x['differences']],
            'actual_inputs_or_plant_results_modified':False,'original_failed_validation_preserved':True,
            'resolution':'deep-copy future control snapshots; uniformly revalidate all8 original trajectories with timestamp_audited tag; compare only the known invalid t0 covariance to same-tick copied trace; no other expected field is replaced',
            'scope':'logging correction, not a new algorithm or a retrospective physical pass'}
    save(ROOT/'logging_integrity_audit.json',result);print(result['affected_runs']);return result

if __name__=='__main__':run()
