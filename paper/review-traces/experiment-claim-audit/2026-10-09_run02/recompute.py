from audit_inspect import *
import numpy as np, gzip
from scipy.optimize import linprog
from scipy.linalg import solve_discrete_are
ROOT=PAPER.parent
BASE=ROOT/'output/fpmfc/n209_paper_system'
HIST=ROOT/'output/fpmfc/n208_adaptive_capture/runs'
def load_npz(p,needed):
    assert p in FILES.values()
    with np.load(p,allow_pickle=False) as z: return {k:z[k] for k in needed}
def maximum(a): return float(np.max(a))
def norm(a): return np.linalg.norm(a,axis=-1)
def clean(d):
    if isinstance(d,dict): return {k:clean(v) for k,v in d.items()}
    if isinstance(d,(list,tuple)): return [clean(v) for v in d]
    if isinstance(d,np.ndarray): return d.tolist()
    if isinstance(d,np.generic): return d.item()
    return d
out={'runs':{},'phase_one':{},'historical':{}}
ledger=read(BASE/'run_ledger.json')
for attempt in ledger['attempts']:
    name=attempt['name']; p=BASE/'runs'/name
    z=load_npz(p/'trace.npz',['time_s','eq_active','local_postlatch_time_s','target_omega_world_rad_s','target_base_relative_omega_world_rad_s','load_fraction','force_grasp_n','moment_grasp_nm','linear_momentum_world_kg_m_s','angular_momentum_about_center_world_kg_m2_s','intentional_contact_count','unintended_contact_count','contact_penetration_m','estimate_covariance','estimate_valid','estimated_capture_values','phase_code','parameter_feedback_used','ctrl_nm','qfrc_applied','xfrc_applied','interface_translation_error_m','interface_rotation_error_deg','measurement_age_s','estimate_output_time_s','estimate_current_tick'])
    t=z['time_s']; m=read(p/'metrics.json'); c=read(p/'config.json'); states=read(p/'control_states.json'); gov=read(p/'progress_governor.json'); val=read(p/'validation_timestamp_audited.json'); events=read(p/'events.json')
    latch_ids=np.flatnonzero(z['eq_active'].reshape(len(t),-1).any(axis=1)); latch=float(t[latch_ids[0]]) if len(latch_ids) else None
    window=None if latch is None else ((t>=latch+18-1e-9)&(t<=latch+20+1e-9))
    nwindow=0 if window is None else int(window.sum())
    covariance_discrepancies=[]
    for s in states:
        idx=int(np.argmin(abs(t-s['time'])))
        assert abs(t[idx]-s['time'])<1e-8
        d=maximum(abs(np.asarray(s['estimate']['covariance'])-z['estimate_covariance'][idx]))
        if d>1e-15: covariance_discrepancies.append({'time':s['time'],'max_diff':d,'valid':s['estimate']['valid']})
    packet_count=0; stamps=[]; arrivals=[]; first_packet=None
    with gzip.open(p/'packets.jsonl.gz','rt',encoding='utf-8') as f:
        for line in f:
            packet=json.loads(line);packet_count+=1
            if first_packet is None:first_packet=packet
    r={'status':m['status'],'category':attempt['category'],'version':attempt['version'],'end':float(t[-1]),'samples':len(t),'dt_max_error':maximum(abs(np.diff(t)-0.002)),'latch':latch,'post_duration':None if latch is None else float(t[-1]-latch),'window_count':nwindow,'window_start':None if not nwindow else float(t[window][0]),'world_window':None if not nwindow else maximum(np.rad2deg(norm(z['target_omega_world_rad_s'][window]))),'relative_window':None if not nwindow else maximum(np.rad2deg(norm(z['target_base_relative_omega_world_rad_s'][window]))),'peak_rho':maximum(z['load_fraction']),'load_formula_max_error':maximum(abs(z['load_fraction']-(norm(z['force_grasp_n'])/50+norm(z['moment_grasp_nm'])/2))),'P_drift':maximum(norm(z['linear_momentum_world_kg_m_s']-z['linear_momentum_world_kg_m_s'][0])),'H_drift':maximum(norm(z['angular_momentum_about_center_world_kg_m2_s']-z['angular_momentum_about_center_world_kg_m2_s'][0])),'first_contact':None if not np.any(z['intentional_contact_count']) else float(t[np.flatnonzero(z['intentional_contact_count'])[0]]),'unintended_contact_max':int(z['unintended_contact_count'].max()),'parameter_feedback_used':bool(z['parameter_feedback_used'].any()),'actuators':z['ctrl_nm'].shape[1],'max_qfrc_applied':maximum(abs(z['qfrc_applied'])),'max_xfrc_applied':maximum(abs(z['xfrc_applied'])),'safety_violation_records':len(m['actual_safety_violations']),'config_capture':c['mission']['capture'],'config_performance':c['mission']['performance'],'config_n209':c['mission']['n209'],'cpu':m['cpu_s'],'latency':m['control_latency_s'],'predictor_max':max(g['latency_s'] for g in gov),'terminal_governor':gov[-1],'covariance_discrepancies':covariance_discrepancies,'packet_count':packet_count,'validator':{k:v for k,v in val.items() if 'identity' not in k},'events':[{k:v for k,v in e.items() if k!='reason'} for e in events],'capture_states':sum(bool(s['gate']) for s in states)}
    r['metric_differences']={k:None if r[rk] is None else r[rk]-m[k] for rk,k in [('end','end_time_s'),('latch','latch_time_s'),('world_window','world_window_max_deg_s'),('relative_window','relative_window_max_deg_s'),('peak_rho','max_load_fraction'),('P_drift','max_P_drift_kg_m_s'),('H_drift','max_H_drift_kg_m2_s')]}
    if 'S01' in name:
        mask=(t>=1-1e-9)&(t<=7+1e-9); cov=z['estimate_covariance'][mask]
        # Largest eigenvalue within each 3x3 block; conservative component lower bound.
        lin=3*np.sqrt(np.linalg.eigvalsh(cov[:,6:9,6:9])[:,-1]); ang=3*np.sqrt(np.linalg.eigvalsh(cov[:,9:12,9:12])[:,-1])
        r['gate_counterfactual']={'samples':int(mask.sum()),'lin_min':float(lin.min()),'ang_min':float(ang.min()),'precluded':int(((lin>0.001)|(ang>np.deg2rad(.2))).sum()),'valid':int(z['estimate_valid'][mask].sum()),'min_age':float(z['measurement_age_s'][mask].min()),'first_packet':first_packet}
    out['runs'][name]=r
for name in ['H2_prior','S01_noise_delay','D05_final_nominal']:
    snapshots=read(BASE/'diagnostics'/name/'snapshots.json');zs=[]
    for snap in snapshots:
        q=snap['qp'][0];A=np.array(q['A']);l=np.array(q['l'],dtype=float);u=np.array([np.inf if v is None else v for v in q['u']]); rows=np.isfinite(u)
        AA=np.vstack([-A,A[rows]]);bb=np.r_[-l,u[rows]]
        result=linprog(np.r_[np.zeros(A.shape[1]),1.],A_ub=np.c_[AA,-np.ones(len(bb))],b_ub=bb,bounds=[(None,None)]*A.shape[1]+[(0,None)],method='highs',options={'primal_feasibility_tolerance':1e-9,'dual_feasibility_tolerance':1e-9})
        assert result.success
        zs.append(float(result.fun))
    snap=snapshots[-1];q=snap['qp'][0];A=np.array(q['A']);l=np.array(q['l']);u=np.array([np.inf if v is None else v for v in q['u']]);dist=5
    distLP=linprog(np.zeros(7),A_ub=-A[:dist],b_ub=-l[:dist],bounds=[(None,None)]*7,method='highs')
    lo=np.array(snap['raw_lower']);hi=np.array(snap['raw_upper']); j=snap['row_names'].index('gripper_contact_pad__tumbling_target_geom');boxmax=float(np.sum(np.where(A[j]>=0,A[j]*hi,A[j]*lo)))
    out['phase_one'][name]={'time':snap['time'],'window':snap['time']-snapshots[0]['time'],'count':len(snapshots),'z':zs[-1],'record_z':q['phase_one']['z'],'all_z':zs,'distance_alone_feasible':bool(distLP.success),'joint_box_feasible':bool(np.all(lo<=hi)),'required':l[j],'box_max':boxmax,'shortfall':float(l[j]-boxmax),'geometry':[{'name':g['name'],'distance_m':g['distance_m'],'d_min_m':g['d_min_m']} for g in snap['geometry']]}
old=load_npz(HIST/'S01_noise_delay/trace.npz',['time_s','estimate_error','estimate_covariance']);new=read(BASE/'diagnostics/S01_noise_delay/new_estimator_redecision.json');nt=np.array([x['time'] for x in new]);ne=np.array([x['error'] for x in new]);ns=np.array([x['sigma'] for x in new]);om=(old['time_s']>=1-1e-9)&(old['time_s']<=7+1e-9);nm=(nt>=1-1e-9)&(nt<=7+1e-9)
out['perception']={'old_count':int(om.sum()),'new_count':int(nm.sum()),'time_max_error':maximum(abs(old['time_s'][om]-nt[nm])),'old_rms':np.sqrt(np.mean(old['estimate_error'][om]**2,axis=0)),'new_rms':np.sqrt(np.mean(ne[nm]**2,axis=0)),'new_coverage':np.mean(abs(ne[nm])<=3*ns[nm],axis=0)}
for scenario in ['H1','H3']:
    needed=['time_s','ctrl_nm','qpos','qvel','parameter_pi','parameter_rank']
    prior=load_npz(HIST/(scenario+'_prior')/'trace.npz',needed);identified=load_npz(HIST/(scenario+'_identified')/'trace.npz',needed)
    config=read(HIST/(scenario+'_prior')/'config.json');ip=read(HIST/(scenario+'_prior')/'identification.json');g0=read(HIST/(scenario+'_prior')/'governor.json');g1=read(HIST/(scenario+'_identified')/'governor.json')
    blocks=load_npz(HIST/(scenario+'_prior')/'regression_blocks.npz',['DY','cholesky']);raw=blocks['DY'].reshape(-1,10);white=np.concatenate([np.linalg.solve(ch,y) for ch,y in zip(blocks['cholesky'],blocks['DY'])]);scaled=white*np.array(config['prior']['parameter_scales'])[None,:];sing=np.linalg.svd(scaled,compute_uv=False)
    out['historical'][scenario]={'torque_max_difference':maximum(abs(prior['ctrl_nm']-identified['ctrl_nm'])),'qpos_max_difference':maximum(abs(prior['qpos']-identified['qpos'])),'qvel_max_difference':maximum(abs(prior['qvel']-identified['qvel'])),'final_com_mm':float(np.linalg.norm(prior['parameter_pi'][-1,1:4]/prior['parameter_pi'][-1,0]-np.array(config['truth_evaluation_only']['com']))*1000),'rank_final':int(prior['parameter_rank'][-1]),'rank_max':int(prior['parameter_rank'].max()),'raw_singular':np.linalg.svd(raw,compute_uv=False),'white_singular':np.linalg.svd(white,compute_uv=False),'scaled_white_singular':sing,'scaled_white_rank':int(np.sum((sing>=config['mission']['information_singular_min'])&(sing>=sing[0]*config['mission']['information_relative_min']))),'reported_singular_max_error':maximum(abs(sing-np.array(ip['final']['singular_values']))),'prediction_parameter_difference':max(np.max(abs(np.array(a['model_pi'])-b['model_pi'])) for a,b in zip(g0,g1))}
# Independent scalar CV covariance calculation, no simulation or repository module imported.
d=.006;age=.012;q=.003**2;R=1e-10;F=np.array([[1.,d],[0,1.]]);C=np.array([[1.,0.]]);Q=q*np.array([[d**3/3,d*d/2],[d*d/2,d]])
pred=solve_discrete_are(F.T,C.T,Q,np.array([[R]]));post=pred-pred@C.T@np.linalg.solve(C@pred@C.T+R,C@pred)
P=np.diag([R,.1])
for it in range(20000):
    Pp=F@P@F.T+Q;Pn=Pp-Pp@C.T@np.linalg.solve(C@Pp@C.T+R,C@Pp)
    if np.max(abs(Pn-P))<1e-22:break
    P=Pn
out['riccati']={'post':post,'iteration_count':it+1,'iteration_dare_error':maximum(abs(Pn-post)),'floor_m_s':float(3*np.sqrt(post[1,1]+q*age)),'initial_minus_fixed_eigen':np.linalg.eigvalsh(np.diag([R,.1])-post),'residual':maximum(abs((F@post@F.T+Q)-(F@post@F.T+Q)@C.T@np.linalg.solve(C@(F@post@F.T+Q)@C.T+R,C@(F@post@F.T+Q))-post))}
out=clean(out)
(HERE/'recomputed_metrics.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps({'runs':{n:{k:r[k] for k in ['status','end','latch','post_duration','window_count','world_window','relative_window','peak_rho','load_formula_max_error','P_drift','H_drift','first_contact','capture_states','covariance_discrepancies','metric_differences']} for n,r in out['runs'].items()},'phase_one':{n:{k:v for k,v in r.items() if k not in ['geometry','all_z']} for n,r in out['phase_one'].items()},'perception':out['perception'],'historical':out['historical'],'riccati':out['riccati']},indent=2))
