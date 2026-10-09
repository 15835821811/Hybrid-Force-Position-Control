"""Offline Phase-I and per-instance recording; never sends elastic LP output."""
import copy
import dataclasses
import gzip
import importlib.util
import json
import sys
import time
import types
import numpy as np
from scipy.linalg import cho_factor, cho_solve
from scipy.optimize import linprog
from .common import ROOT, OLD, PROJECT_ROOT, read, save, sha, charge

def phase_one(A, lower, upper, scales=None):
    A=np.asarray(A,float); lower=np.asarray(lower,float); upper=np.asarray(upper,float)
    scales=np.ones(len(A)) if scales is None else np.asarray(scales,float)
    if np.any(~np.isfinite(scales)) or np.any(scales<=0):raise ValueError('positive finite row scales required')
    lo=np.isfinite(lower);hi=np.isfinite(upper)
    M=np.vstack([np.c_[-A[lo],-scales[lo]],np.c_[A[hi],-scales[hi]]])
    b=np.r_[-lower[lo],upper[hi]]
    result=linprog(np.r_[np.zeros(A.shape[1]),1.],A_ub=M,b_ub=b,
                   bounds=[(None,None)]*A.shape[1]+[(0,None)],method='highs',
                   options={'primal_feasibility_tolerance':1e-9,'dual_feasibility_tolerance':1e-9})
    out={'solver':'SciPy/HiGHS LP','status':result.message,'scales':scales.tolist(),'tolerance':1e-9,
         'classification':'UNRESOLVED_NUMERICAL_CLASSIFICATION'}
    if result.success:
        x=result.x[:-1];v=A@x;violation=np.maximum(np.maximum(lower-v,v-upper),0.)
        out.update(z=float(result.x[-1]),x=x.tolist(),row_violation_original_units=violation.tolist(),
                   relaxed_residual=float(np.max(violation-result.x[-1]*scales)),
                   classification='LINEAR_HARD_SET_FEASIBLE' if result.x[-1]<=1e-8 and max(violation)<=1e-8 else 'LINEAR_HARD_SET_INFEASIBLE',
                   dual_inequality=result.ineqlin.marginals.tolist())
    return out

def finite_list(a):
    a=np.asarray(a)
    return [None if not np.isfinite(x) else float(x) for x in a] if a.ndim==1 else [finite_list(x) for x in a]

def recorded_admm(self,H,g,A,l,u,initial):
    # Same operations and stopping test as historical solver; record actual dual.
    cfg=self.config;factor=cho_factor(H+cfg.admm_sigma*np.eye(7)+cfg.admm_rho*A.T@A,lower=True,check_finite=False)
    x=initial.copy();v=A@x;z=np.minimum(np.maximum(v,l),u);y=np.zeros(len(l));status='maximum_iterations'
    for k in range(1,cfg.qp_max_iterations+1):
        x=cho_solve(factor,cfg.admm_sigma*x-g+A.T@(cfg.admm_rho*z-y),check_finite=False)
        v=A@x;rel=cfg.admm_relaxation*v+(1-cfg.admm_relaxation)*z
        z=np.minimum(np.maximum(rel+y/cfg.admm_rho,l),u);y+=cfg.admm_rho*(rel-z)
        primal=float(max(abs(v-z)));dual=float(max(abs(H@x+g+A.T@y)))
        if primal<=cfg.qp_tolerance*max(1.,float(max(abs(v)))) and dual<=5*cfg.qp_tolerance*max(1.,float(max(abs(g)))):status='solved';break
    feasible=bool(min(np.minimum(v-l,u-v))>=-cfg.feasibility_tolerance)
    if feasible and status!='solved':status='solved_inaccurate'
    if self.n209_record:
        self.n209_qps.append({'H':H.copy(),'g':g.copy(),'A':A.copy(),'l':finite_list(l),'u':finite_list(u),
                              'x':x.copy(),'dual':y.copy(),'initial':initial.copy(),'status':status,'iterations':k,
                              'primal_residual':primal,'stationarity_inf':dual,'feasible':feasible,
                              'eigenvalues_H':np.linalg.eigvalsh(H),'singular_values_A':np.linalg.svd(A,compute_uv=False),
                              'phase_one':phase_one(A,l,u)})
    return x,feasible,status,k

def attach_recorder(hqp):
    hqp.n209_record=False;hqp.n209_qps=[];hqp.n209_secondary=None
    hqp._solve_admm=types.MethodType(recorded_admm,hqp)
    original=hqp._solve_scalar_nullspace_secondary
    def secondary(self,primary,J,H,g,A,l,u):
        result=original(primary,J,H,g,A,l,u)
        if self.n209_record:
            achieved=J@primary;tol=np.r_[[self.fpmfc_config.level1_position_tolerance_m_s]*3,[self.fpmfc_config.level1_angular_tolerance_rad_s]*3]
            self.n209_secondary={'H':H.copy(),'g':g.copy(),'J_lock':J.copy(),'achieved':achieved,'lock_tolerance':tol,
                'x':result[0].copy(),'success':result[1],'status':result[2], 'iterations':result[3],
                'phase_one':phase_one(np.vstack([A,J]),np.r_[l,achieved-tol],np.r_[u,achieved+tol]),
                'J_singular_values':np.linalg.svd(J,compute_uv=False)}
        return result
    hqp._solve_scalar_nullspace_secondary=types.MethodType(secondary,hqp)

def historical_controller(cfg,name):
    # Separate process per history; restore exact archived online modules.
    for file in ['contracts','momentum_regressor','known_model','state_estimator','inertial_estimator','relative_reference','risk','information_gate','governor','controller']:
        key='v6_mujoco/adaptive_capture/'+file+'.py';p=OLD/'source_versions'/name/(file+'.py')
        if sha(p)!=cfg['identity'][key]:raise RuntimeError('historical source identity mismatch '+key)
        module_name='v6_mujoco.adaptive_capture.'+file
        spec=importlib.util.spec_from_file_location(module_name,p);module=importlib.util.module_from_spec(spec)
        sys.modules[module_name]=module;spec.loader.exec_module(module)
    # Shared dependencies must also match original identity.
    for p in ['v6_mujoco/fpmfc/controller.py','v6_mujoco/hierarchical_qp.py','v6_mujoco/geometry_capture/planning.py']:
        if sha(PROJECT_ROOT/p)!=cfg['identity'][p]:raise RuntimeError('shared historical dependency changed '+p)
    return sys.modules['v6_mujoco.adaptive_capture.controller'].Controller(cfg['prior'],cfg['mission'])

def audit(name):
    cpu=time.process_time();wall=time.perf_counter();out=ROOT/'diagnostics'/name
    if (out/'diagnosis.json').exists():raise FileExistsError(out)
    cfg=read(OLD/'runs'/name/'config.json');end=read(OLD/'runs'/name/'metrics.json')['end_time_s']
    c=historical_controller(cfg,name);attach_recorder(c.hqp)
    from v6_mujoco.adaptive_capture.validate import packet_from_dict
    from v6_mujoco.geometry_capture.planning import full_distance_gradient
    from v6_mujoco.geometry_capture.design import planning_minimum
    import mujoco
    ticks=[];maxerr=0.;events_match=True;stream=[];last_ref=None;last_meas=None
    with gzip.open(OLD/'runs'/name/'packets.jsonl.gz','rt',encoding='utf-8') as f:
        for line in f:
            row=json.loads(line);p=packet_from_dict(row['packet']);task=round(p.time/cfg['mission']['servo_s'])%round(cfg['mission']['task_s']/cfg['mission']['servo_s'])==0
            record=task and p.time>=end-.5000001
            c.hqp.n209_record=record;c.hqp.n209_qps=[];before_v=c.hqp.previous_velocity.copy();before_a=c.hqp.previous_acceleration.copy()
            tau,latch=c.update(p,task);maxerr=max(maxerr,float(max(abs(tau-row['torque']))));events_match &= latch==row['latch']
            if task and c.latest_reference is not None:
                e=c.estimate;r=c.latest_reference
                stream.append({'time':p.time,'age':p.time-e.measurement_time,'new_measurement':e.measurement_time!=last_meas,
                    'estimate_p':e.p,'estimate_v':e.v,'estimate_w':e.w,'reference_p':r.position_world_m,'reference_v':r.linear_velocity_world_m_s,
                    'reference_w':r.angular_velocity_world_rad_s,'position_step':None if last_ref is None else np.linalg.norm(r.position_world_m-last_ref),
                    'progress':c.reference.progress,'rate':c.reference.rate,'covariance_diag':np.diag(e.covariance)})
                last_ref=r.position_world_m.copy();last_meas=e.measurement_time
            if record and c.hqp.n209_qps:
                mapping,_=c.hqp.reaction_velocity_map(c.data);drift=c.data.qvel-mapping@c.data.qvel[c.vids];geometry=[]
                names=[x['pair'] for x in c.hqp.last_geometry]
                for pair in c.hqp.pairs:
                    if pair.name not in names:continue
                    grad=full_distance_gradient(c.model,c.data,pair);points=np.zeros(6)
                    dist=mujoco.mj_geomDistance(c.model,c.data,pair.geom_a,pair.geom_b,.14,points)
                    norm=np.linalg.norm(points[3:]-points[:3]);normal=(points[3:]-points[:3])/max(norm,1e-12)
                    geometry.append({'name':pair.name,'category':pair.category,'distance_m':dist,'d_min_m':planning_minimum(pair),
                        'points_world':points,'segment_normal':normal,'normal_convention':'reported geomDistance segment; full signed FD determines row',
                        'full_gradient':grad,'mapped_gradient':grad@mapping,'drift_m_s':float(grad@drift),'drift_generalized':drift})
                ticks.append({'time':p.time,'tasks':c.tasks[-1], 'qpos':c.data.qpos.copy(),'qvel':c.data.qvel.copy(),
                    'previous_velocity':before_v,'previous_acceleration':before_a,'raw_lower':c.hqp.last_raw_velocity_lower.copy(),
                    'raw_upper':c.hqp.last_raw_velocity_upper.copy(),'geometry':geometry,'qp':c.hqp.n209_qps,
                    'row_names':names+['joint_'+str(i+1) for i in range(7)],'row_units':['m/s']*len(names)+['rad/s']*7,
                    'shape':dataclasses.asdict(c.shape.sample(c.data)),'secondary':c.hqp.n209_secondary,
                    'estimator':dataclasses.asdict(c.estimate),'reference':dataclasses.asdict(c.latest_reference),
                    'reference_internal':{k:getattr(c.reference,k) for k in ['progress','rate','accel','jerk','z1','z2','tau']}})
    save(out/'snapshots.json',ticks);save(out/'sensor_reference.json',stream)
    final=ticks[-1];summary={'run':name,'end_s':end,'new_robot_attempts':0,'source_config_sha':sha(OLD/'runs'/name/'config.json'),
        'snapshot_sha':sha(out/'snapshots.json'),'decision_torque_max_error':maxerr,'decision_events_match':events_match,
        'exact_historical_reconstruction':bool(maxerr<1e-10 and events_match),'final_phase_one':final['qp'][0]['phase_one'],
        'final_solver':{k:final['qp'][0][k] for k in ['status','iterations','primal_residual','stationarity_inf','feasible']},
        'final_shape':final['shape'],'final_secondary_status':final['secondary']['status'],
        'row_names':final['row_names'],'units':final['row_units'],'final_time':final['time'],
        'cpu_s':time.process_time()-cpu,'wall_s':time.perf_counter()-wall}
    save(out/'diagnosis.json',summary);charge('historical_packet_diagnostic',summary['cpu_s'],summary['wall_s'],name)
    print(json.dumps(summary,default=lambda x: x.tolist() if hasattr(x,'tolist') else str(x)),flush=True)

if __name__=='__main__':audit(sys.argv[1])
