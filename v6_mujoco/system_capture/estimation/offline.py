"""Predeclared synthetic calibration, held-out signals and archival regression.

Truth is used only by this offline experiment/evaluator, never filter inputs.
No new complete robot trajectory is generated here.
"""
import copy
import gzip
import json
import time
from dataclasses import replace
import numpy as np
from scipy.integrate import solve_ivp
from scipy.spatial.transform import Rotation
from v6_mujoco.adaptive_capture.contracts import PoseObservation,skew
from v6_mujoco.adaptive_capture.validate import packet_from_dict
from v6_mujoco.feasible_capture.estimator_adapter import TimedEstimator
from ..common import save,sha,clean
from .common import OUT,OLD,PROJECT,RUNS,read,charge
from .filter import AccelerationEstimator,predict_snapshot,white_jerk
from .budget import diagnostic,guard_margins,limits
from .tests import packet

KINDS=('constant','multiaxis','offset_COM','smooth_transition','jitter_dropout','bias_noise')
DT=.002
OFFSET=np.array([0.,0.,.15])  # Replaced by actual public site transform below.

def public_offset():
    from v6_mujoco.adaptive_capture.known_model import compile_known
    from v6_mujoco.model import site_id
    cfg=read(OLD/'runs/R03_V2_S01/config.json')
    m,d=compile_known(cfg['prior'],cfg['mission']['solver_tolerance'])
    return m.site_pos[site_id(m,'target_grasp_site')].copy()

def preregister():
    path=OUT/'calibration_split.json'
    if path.exists():return read(path)
    split=dict(schema='S02_calibration_v1',frozen_epoch=time.time(),
        development=dict(seeds=[52020+i for i in range(6)],interval_s=[0.,2.],usage='two candidate calibration and selection only'),
        validation=dict(seeds=[52120+i for i in range(6)],interval_s=[2.,6.],
            usage='frozen before parameter selection; first 2 s causal warmup excluded from validation summaries, all prefixes retained'),
        archived=dict(runs=list(RUNS),usage='already-seen regression only; never tuning'),
        families=list(KINDS),signal_dt_s=DT,formal_new_noise_seed=210102,
        candidates=dict(count=2,structure='18-state CA SO3 left error',
            density_rule='4x and 8x maximum-axis RMS acceleration increments / sqrt(dt), normalized by declared contact multiplier, on independent development signals only',
            selection='min mean of normalized current-state squared error + blind 12ms squared error + 0.02*abs(log(NIS/6)) + 2*max(0,0.95-minimum marginal 3sigma coverage); no covariance-only selection'),
        initialization=dict(velocity_mean=[0,0,0],omega_mean=[0,0,0],accel_mean=[0,0,0],alpha_mean=[0,0,0],
            initial_velocity_variance=.1,initial_omega_variance=.1,initial_accel_variance=.01,initial_alpha_variance=1.,minimum_samples=3),
        independence='six deterministic signal families and separate noise seeds; temporally correlated ticks are not independent experiments',
        excitation='mathematical trajectories, including prescribed smooth acceleration; not driven targets in formal robot runs')
    save(path,split)
    return split

def signal(kind,duration):
    t=np.arange(round(duration/DT)+1)*DT
    mult=kind!='constant'
    def omega(u):
        if not mult:return np.array([0,0,.035])
        return np.array([.03+.035*np.sin(1.3*u),.04*np.cos(.9*u),.02+.025*np.sin(1.1*u)])
    def alpha(u):
        if not mult:return np.zeros(3)
        return np.array([.0455*np.cos(1.3*u),-.036*np.sin(.9*u),.0275*np.cos(1.1*u)])
    smooth=kind in ('smooth_transition','bias_noise')
    def acceleration(u):
        if not smooth:return np.zeros(3)
        return np.array([.004,-.002,.001])*(np.tanh((u-1.2)/.12)-np.tanh((u-3.8)/.12))/2
    def ode(u,y):return np.r_[y[3:6],acceleration(u),(skew(omega(u))@y[6:].reshape(3,3)).ravel()]
    y0=np.r_[[.4,.2,.1],[.006,-.004,.002],np.eye(3).ravel()]
    y=solve_ivp(ode,[0,duration],y0,t_eval=t,rtol=2e-11,atol=1e-13).y.T
    p=y[:,:3].copy();v=y[:,3:6].copy();R=Rotation.from_matrix(y[:,6:].reshape(-1,3,3)).as_matrix()
    w=np.array([omega(u) for u in t]);al=np.array([alpha(u) for u in t]);a=np.array([acceleration(u) for u in t])
    if kind in ('offset_COM','jitter_dropout','bias_noise'):
        r=R@np.array([.055,-.035,.025]);p-=r;v-=np.cross(w,r);a-=np.cross(al,r)+np.cross(w,np.cross(w,r))
    contact=(t>=1.2)&(t<=4.2) if smooth else np.zeros(len(t),bool)
    return dict(time=t,p=p,R=R,v=v,w=w,a=a,alpha=al,contact=contact)

def make_packets(truth,kind,seed,scfg):
    t=truth['time'];queue=[];packets=[];sample_index=0
    for i,now in enumerate(t):
        if i%3==0:
            absolute_index=i//3
            rng=lambda ch:np.random.default_rng(np.random.SeedSequence([seed,absolute_index,ch])).normal(size=3)
            drop=kind=='jitter_dropout' and absolute_index%71 in (30,31,32)
            jitter=(absolute_index%3)*DT if kind=='jitter_dropout' else 0.
            if not drop:
                obs=PoseObservation(float(now),truth['p'][i]+scfg['position_sigma_m']*rng(1)+scfg['position_bias_m'],
                    Rotation.from_rotvec(scfg['rotation_sigma_rad']*rng(2)+scfg['rotation_bias_rad']).as_matrix()@truth['R'][i],
                    np.diag([scfg['position_sigma_m']**2]*3+[scfg['rotation_sigma_rad']**2]*3))
                queue.append((now+scfg['delay_s']+jitter,obs,absolute_index))
        delivered=[q for q in queue if q[0]<=now+1e-10];queue=[q for q in queue if q[0]>now+1e-10]
        packets.append(replace(packet(float(now),contact=bool(truth['contact'][i])),poses=tuple(q[1] for q in delivered)))
    return packets

def error(e,truth,i):
    return np.r_[e.p-truth['p'][i],Rotation.from_matrix(e.R@truth['R'][i].T).as_rotvec(),e.v-truth['v'][i],e.w-truth['w'][i]]

def stats(errors,P,mask):
    e=errors[mask];cov=P[mask]
    if not len(e):return dict(status='NOT_EVALUATED')
    sig=np.sqrt(np.maximum(np.diagonal(cov,axis1=1,axis2=2),1e-30))
    nees=np.einsum('bi,bi->b',e,np.linalg.solve(cov,e[...,None])[...,0])
    return dict(samples=len(e),component_rms=np.sqrt(np.mean(e**2,axis=0)),component_peak=np.max(abs(e),axis=0),
        norm_rms=[np.sqrt(np.mean(np.sum(e[:,k:k+3]**2,axis=1))) for k in (0,3,6,9)],
        norm_peak=[np.max(np.linalg.norm(e[:,k:k+3],axis=1)) for k in (0,3,6,9)],
        marginal_3sigma_coverage=np.mean(abs(e)<=3*sig,axis=0),NEES12_mean=np.mean(nees),NEES12_peak=np.max(nees),
        covariance_scope='12-dimensional left-error covariance; correlated time samples, no population confidence or joint safety certification')

def evaluate(name,filter_,packets,truth,interval,cfg,offset,save_raw=True):
    n=len(packets);errors=np.zeros((n,12));P=np.zeros((n,12,12));margins=np.zeros((n,4));full=np.zeros((n,4));valid=np.zeros(n,bool)
    past=[];blind=[];blind_times=[];age=[];all_est=[];started=time.perf_counter()
    for i,pkt in enumerate(packets):
        e=filter_.update(pkt);all_est.append(e);valid[i]=e.valid;P[i]=e.covariance;errors[i]=error(e,truth,i)
        b=diagnostic(e,cfg,offset);margins[i]=b['legacy_margin'];full[i]=b['complete_geometry_margin'];age.append(pkt.time-e.measurement_time)
        if e.valid and interval[0]-1e-10<=pkt.time<=interval[1]+1e-10:
            if isinstance(filter_,AccelerationEstimator):post=filter_.s
            else:
                post=replace(e,time=filter_.time,p=filter_.p,R=filter_.R,v=filter_.v,w=filter_.w,covariance=filter_.P)
            k=int(round(post.time/DT))
            if k<len(packets):past.append(error(post,truth,k))
            # No observations are assimilated in a 0--max-delay blind forecast.
            if i%3==0:
                for h in (0.,.006,.012,.016):
                    k=i+int(round(h/DT))
                    if k>=n:continue
                    if isinstance(filter_,AccelerationEstimator):forecast=predict_snapshot(e,e.time+h)
                    else:
                        vals=filter_.propagate(e.p,e.R,e.v,e.w,e.covariance,h)
                        forecast=replace(e,time=e.time+h,p=vals[0],R=vals[1],v=vals[2],w=vals[3],covariance=vals[4])
                    blind.append(error(forecast,truth,k));blind_times.append(h)
    elapsed=time.perf_counter()-started
    t=truth['time'];mask=valid&(t>=interval[0]-1e-10)&(t<=interval[1]+1e-10)
    result=stats(errors,P,mask);result['interval_s']=interval;result['all_prefix']=stats(errors,P,valid)
    result['past_measurement_time']=dict(component_rms=np.sqrt(np.mean(np.array(past)**2,axis=0)),component_peak=np.max(abs(np.array(past)),axis=0)) if past else None
    result['blind_prediction']={str(h):dict(component_rms=np.sqrt(np.mean(np.array(blind)[np.array(blind_times)==h]**2,axis=0)),component_peak=np.max(abs(np.array(blind)[np.array(blind_times)==h]),axis=0)) for h in set(blind_times)}
    logs=[x for x in filter_.log if interval[0]<=x['arrival']<=interval[1]]
    inn=np.array([x['innovation'] for x in logs]);nis=np.array([x['NIS'] for x in logs])
    result['innovation']=dict(count=len(logs),NIS6_mean=float(np.mean(nis)) if len(nis) else None,NIS6_peak=float(max(nis)) if len(nis) else None,
        lag1_correlation=[float(np.corrcoef(inn[:-1,j],inn[1:,j])[0,1]) if min(np.std(inn[:-1,j]),np.std(inn[1:,j]))>1e-30 else None for j in range(6)] if len(inn)>2 else None,
        correlation_undefined_policy='null for constant or insufficient innovations',
        component_mean=inn.mean(axis=0) if len(inn) else None)
    result['information']=dict(legacy_margin_min=margins[mask].min(axis=0),legacy_margin_max=margins[mask].max(axis=0),
        necessary_condition_fraction=float(np.mean(np.all(margins[mask]<limits(cfg),axis=1))),
        complete_geometry_condition_fraction=float(np.mean(np.all(full[mask]<limits(cfg),axis=1))),
        actual_guard_calls=0,scope='counterfactual necessary conditions; no robot capture guard invoked')
    result['timing']=dict(total_wall_s=elapsed,updates=n,rejected_packets=len(filter_.rejects))
    if hasattr(filter_,'update_times'):result['timing']['update_quantiles_s']=np.quantile(filter_.update_times,[.5,.95,.99,1.])
    if save_raw:
        dest=OUT/'offline'/name;dest.mkdir(parents=True,exist_ok=True)
        np.savez_compressed(dest/'comparison.npz',time=t,error=errors,P=P,valid=valid,age=age,legacy_margin=margins,
            legacy_remaining_budget=limits(cfg)-margins,full_geometry_margin=full,full_remaining_budget=limits(cfg)-full,
            blind_error=blind,blind_horizon=blind_times,innovation=inn,NIS=nis,
            estimated_p=np.array([e.p for e in all_est]),estimated_R=np.array([e.R for e in all_est]),
            estimated_v=np.array([e.v for e in all_est]),estimated_w=np.array([e.w for e in all_est]))
        save(dest/'metrics.json',clean(result));result['raw_sha256']=sha(dest/'comparison.npz')
    return clean(result)

def calibrate():
    split=preregister();cpu,wall=time.process_time(),time.perf_counter()
    cfg=read(OLD/'runs/R03_V2_S01/config.json');scfg=cfg['sensors'];offset=public_offset()
    signals={kind:signal(kind,2.) for kind in KINDS}
    densities=[]
    for tr in signals.values():
        factor=np.where(tr['contact'][:-1],10.,1.)[:,None]
        densities.append([np.sqrt(np.mean((np.diff(tr[key],axis=0)/factor)**2,axis=0)/DT).max() for key in ('a','alpha')])
    scale=np.max(densities,axis=0)
    candidates={};evidence={}
    for j,mult in enumerate((4.,8.)):
        model=dict(linear_jerk_density=float(mult*scale[0]),angular_jerk_density=float(mult*scale[1]),contact_multiplier=10.,
            **{k:v for k,v in split['initialization'].items() if not k.endswith('_mean')},max_step_s=.002)
        name='CA'+str(j+1);c=copy.deepcopy(cfg['mission']);c['s02_estimator']=model;candidates[name]=model;rows={};scores=[]
        for i,kind in enumerate(KINDS):
            packets=make_packets(signals[kind],kind,split['development']['seeds'][i],scfg)
            r=evaluate('development_'+name+'_'+kind,AccelerationEstimator(c),packets,signals[kind],[.2,2.],c,offset)
            rows[kind]=r
            normal=np.repeat(limits(c),3)
            current=np.mean((np.array(r['component_rms'])/normal)**2)
            future=np.mean((np.array(r['blind_prediction']['0.012']['component_rms'])/normal)**2)
            scores.append(current+future+.02*abs(np.log(max(r['innovation']['NIS6_mean'],1e-20)/6))+2*max(0,.95-min(r['marginal_3sigma_coverage'])))
        evidence[name]=dict(model=model,by_family=rows,score=float(np.mean(scores)))
    selected=min(evidence,key=lambda x:evidence[x]['score'])
    selection=dict(selected=selected,model=candidates[selected],candidates=evidence,
        calibration_density_estimates=densities,base_density=scale,selection_epoch=time.time(),split_sha256=sha(OUT/'calibration_split.json'))
    save(OUT/'calibration_selection.json',selection)
    save(PROJECT/'configs/system_capture/algorithms/s02_acceleration.json',candidates[selected])
    charge('development_calibration',cpu,wall,signal_families=6,candidates=2,new_robot_steps=0,estimator_streams=12)
    return selection

def synthetic_validation(model):
    split=read(OUT/'calibration_split.json');cpu,wall=time.process_time(),time.perf_counter()
    cfg=read(OLD/'runs/R03_V2_S01/config.json');c=cfg['mission'];c['s02_estimator']=model;offset=public_offset();rows={}
    for i,kind in enumerate(KINDS):
        truth=signal(kind,6.);packets=make_packets(truth,kind,split['validation']['seeds'][i],cfg['sensors'])
        dest=OUT/'offline'/('signal_'+kind);dest.mkdir(parents=True,exist_ok=True)
        np.savez_compressed(dest/'truth.npz',**truth)
        with gzip.open(dest/'packets.jsonl.gz','wt',encoding='utf-8') as f:
            for n,p in enumerate(packets):f.write(json.dumps(dict(absolute_control_index=n,packet=clean(p)))+'\n')
        rows[kind]={}
        for name,est in [('CV',TimedEstimator(c)),('CA',AccelerationEstimator(c))]:
            rows[kind][name]=evaluate('validation_'+kind+'_'+name,est,packets,truth,[2.,6.],c,offset)
        print('validated '+kind,flush=True)
    save(OUT/'synthetic_validation.json',rows)
    charge('heldout_synthetic_validation',cpu,wall,signal_families=6,estimator_streams=12,new_robot_steps=0)
    return rows

def historical_validation(model):
    cpu,wall=time.process_time(),time.perf_counter();rows={};offset=public_offset()
    for run in RUNS:
        d=OLD/'runs'/run;cfg=read(d/'config.json');c=cfg['mission'];c['s02_estimator']=model
        with np.load(d/'trace.npz') as z:
            truth=dict(time=z['time_s'],p=z['target_position_world_m'],R=z['target_rotation_world'],
                v=z['target_geometric_twist_world'][:,:3],w=z['target_geometric_twist_world'][:,3:])
        with gzip.open(d/'packets.jsonl.gz','rt',encoding='utf-8') as f:
            packets=[packet_from_dict(json.loads(line)['packet']) for line in f]
        assert len(packets)==len(truth['time'])
        rows[run]={}
        for name,est in [('CV',TimedEstimator(c)),('CA',AccelerationEstimator(c))]:
            path=OUT/'offline'/('archived_'+run+'_'+name)/'metrics.json'
            if path.exists():
                rows[run][name]=read(path)
                rows[run][name]['raw_sha256']=sha(path.parent/'comparison.npz')
            else:
                rows[run][name]=evaluate('archived_'+run+'_'+name,est,packets,truth,[.2,float(truth['time'][-1])],c,offset)
        rows[run]['usage']='already-seen regression, exact original packet stream, no parameter tuning'
        print('archival estimator comparison '+run,flush=True)
    save(OUT/'historical_estimator_comparison.json',rows)
    charge('archival_estimator_only_redecision',cpu,wall,streams=8,new_robot_steps=0)
    return rows

def floor(model):
    cpu,wall=time.process_time(),time.perf_counter()
    from scipy.linalg import solve_discrete_are
    h=.006;delay=.012;R=1e-10;density=.003;F=np.array([[1,h],[0,1.]])
    Q=density**2*np.array([[h**3/3,h*h/2],[h*h/2,h]])
    prior=solve_discrete_are(F.T,np.array([[1.,0.]]).T,Q,np.array([[R]]))
    post=prior-np.outer(prior[:,0],prior[0,:])/(prior[0,0]+R)
    old=3*np.sqrt(post[1,1]+density*density*delay)
    cfg=read(OLD/'runs/R03_V2_S01/config.json');c=cfg['mission'];c['s02_estimator']=model
    f=AccelerationEstimator(c);P=f.P.copy();H=np.zeros((6,18));H[:6,:6]=np.eye(6);M=np.diag([1e-10]*3+[1.225e-9]*3)
    # Numeric periodic recursion, fixed zero angular rate, separate free/contact.
    result={}
    for mode in (False,True):
        s=replace(f.s,full_covariance=P,covariance=P[:12,:12],process_contact=mode)
        from .filter import propagate
        for _ in range(3000):
            s=propagate(s,h,mode);P0=s.full_covariance;K=np.linalg.solve(H@P0@H.T+M,H@P0).T;A=np.eye(18)-K@H
            Pn=A@P0@A.T+K@M@K.T;s=replace(s,full_covariance=Pn,covariance=Pn[:12,:12])
        margins=[]
        for age in (.012,.014,.016):margins.append(guard_margins(propagate(s,age,mode),c,public_offset()))
        result['contact' if mode else 'free']=dict(ages_s=[.012,.014,.016],margins=margins,posterior_P18=Pn)
    out=dict(old_CV=dict(recomputed_linear_3sigma_m_s=old,archived_m_s=read(OLD/'uncertainty_floor.json')['minimum_velocity_3sigma_lower_bound_m_s']),
        new_CA=dict(model=model,numerical_periodic_diagnostic=result,
            scope='zero-rate fixed protocol numerical ranges only; not a nonlinear global DARE theorem or actual error lower bound'))
    save(OUT/'floor_before_after.json',out);charge('covariance_periodic_diagnostic',cpu,wall,recursion_updates=6000,new_robot_steps=0)
    return out
