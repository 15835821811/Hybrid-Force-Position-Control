"""Kinematic screening and real torque-driven SRS integration, kept separate."""
from dataclasses import dataclass
import time
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from .common import assumptions,wrap
from .model import compile_model,initial
from .kinematics import IndependentSRS
from .reference import Reference,target,integrate_configuration
from .control import source_velocity,SRSHQP,servo_torque
from .registry import model as registered_model,controller as registered_controller
from ..contracts import ControllerSetup

@dataclass
class Evaluation:
    metrics:dict
    trace:dict|None=None
    @property
    def objective(self):return self.metrics['ranking_objective']
    @property
    def feasible(self):return self.metrics['feasible']
    def to_dict(self):return self.metrics

def reference_for(k,q,T,psi):return Reference(k.terms(q),k.shape(q[7:]).angle,T,psi)

def sample(k,q,v,ref,time_s,torque,command,hierarchy):
    t=k.terms(q);s=k.shape(q[7:]);twist=t.J@v
    pe=t.p-ref.T[:3,3];re=Rotation.from_matrix(t.R@ref.T[:3,:3].T).as_rotvec()
    return {'time_s':time_s,'qpos':q.copy(),'qvel':v.copy(),'torque_nm':torque.copy(),'command_rad_s':command.copy(),
      'position_error_m':pe,'rotation_error_rad':re,'shape_error_rad':float(wrap(s.angle-ref.psi)),
      'linear_velocity_error_m_s':twist[:3]-ref.v,'angular_velocity_error_rad_s':twist[3:]-ref.w,'shape_velocity_error_rad_s':float(s.J@v[6:]-ref.psidot),
      'base_linear_velocity_world_m_s':v[:3].copy(),'base_angular_velocity_world_rad_s':t.base_rotation@v[3:6],
      'base_rotation_world':t.base_rotation,'flange_position_world_m':t.p,'flange_rotation_world':t.R,
      'reference_position_world_m':ref.T[:3,3].copy(),'reference_rotation_world':ref.T[:3,:3].copy(),
      'shape_rad':s.angle,'shape_reference_rad':ref.psi,'momentum_world_origin':t.momentum_matrix@v,'energy_j':float(.5*v@t.M@v),
      'hierarchy_linear_residual_m_s':float(hierarchy[0]),'hierarchy_angular_residual_rad_s':float(hierarchy[1]),
      'minimum_centerline_distance_m':centerline_distance(t.fk),'physical_clearance_valid':False}

def centerline_distance(f):
    # Sampling of non-adjacent centerline segments, an advisory geometric metric.
    # Not a collision-envelope or mass-support clearance qualification.
    segments=[np.linspace(f.origins[i],f.ends[i],11) for i in (0,2,4,6)]
    values=[]
    for i in range(4):
        for j in range(i+2,4):values.append(np.linalg.norm(segments[i][:,None]-segments[j][None,:],axis=2).min())
    return float(min(values))

def summarize(rows,T,psi,kind,reason,cpu,wall,dt):
    trace={key:np.asarray([row[key] for row in rows]) for key in rows[0]};a=assumptions();g=a['gates'];c=a['controller'];t=trace['time_s'];complete=bool(reason is None and abs(t[-1]-T)<1e-8)
    norm=lambda x:np.linalg.norm(trace[x],axis=1)
    peakw=float(norm('base_angular_velocity_world_rad_s').max());rmsw=float(np.sqrt(np.trapz(norm('base_angular_velocity_world_rad_s')**2,t)/max(t[-1],1e-12)))
    R0=trace['base_rotation_world'][0];drift=Rotation.from_matrix(np.einsum('nij,jk->nik',trace['base_rotation_world'],R0.T)).magnitude()
    initial_terms=IndependentSRS().terms(trace['qpos'][0]);mass=IndependentSRS().mass
    center=sum(m*b[0] for m,b in zip(mass,initial_terms.bodies))/mass.sum();p,_,v,_=target(T);radial=p-center
    alpha=float(np.arccos(np.clip(v@radial/(np.linalg.norm(v)*np.linalg.norm(radial)),-1,1)))
    objective=(peakw/a['optimization']['base_scale_rad_s'])**2+(alpha/a['optimization']['alignment_scale_rad'])**2
    values={
      'path_position_error_m':float(norm('position_error_m').max()),'path_rotation_error_rad':float(norm('rotation_error_rad').max()),'path_shape_error_rad':float(abs(trace['shape_error_rad']).max()),
      'terminal_position_error_m':float(norm('position_error_m')[-1]),'terminal_rotation_error_rad':float(norm('rotation_error_rad')[-1]),'terminal_shape_error_rad':float(abs(trace['shape_error_rad'][-1])),
      'terminal_linear_velocity_error_m_s':float(norm('linear_velocity_error_m_s')[-1]),'terminal_angular_velocity_error_rad_s':float(norm('angular_velocity_error_rad_s')[-1]),
      'linear_momentum_drift_kg_m_s':float(np.linalg.norm(trace['momentum_world_origin'][:,:3]-trace['momentum_world_origin'][0,:3],axis=1).max()),
      'angular_momentum_drift_world_origin_kg_m2_s':float(np.linalg.norm(trace['momentum_world_origin'][:,3:]-trace['momentum_world_origin'][0,3:],axis=1).max())}
    checks={key:{'value':val,'limit':g[key],'units':key.rsplit('_',1)[-1],'passed':bool(val<=g[key])} for key,val in values.items()}
    extras={'joint_position':(float(abs(trace['qpos'][:,7:]).max()),c['joint_position_limit_rad'],'rad'),
      'joint_velocity':(float(abs(trace['qvel'][:,6:]).max()),c['joint_velocity_limit_rad_s'],'rad/s'),
      'torque':(float(abs(trace['torque_nm']).max()),c['torque_limit_nm'],'N m')}
    for key,(val,lim,units) in extras.items():checks[key]={'value':val,'limit':lim,'units':units,'passed':bool(val<=lim+1e-9)}
    violation=sum(max(0,x['value']/x['limit']-1)**2 for x in checks.values())+(0 if complete else 100)
    feasible=complete and all(x['passed'] for x in checks.values())
    metrics={'kind':kind,'T_s':float(T),'psi_f_rad':float(psi),'completed_horizon':complete,'status':'COMPLETED' if complete else reason or 'INCOMPLETE','feasible':bool(feasible),
      'feasibility_scope':'declared inertial SRS and explicit joint/tracking/momentum gates; physical collision clearance is SOURCE_LIMITED',
      'objective_G':float(objective),'ranking_objective':float(objective+1000*violation),'normalized_violation_sum':float(violation),'base_peak_angular_speed_rad_s':peakw,'base_rms_angular_speed_rad_s':rmsw,
      'base_peak_linear_speed_m_s':float(norm('base_linear_velocity_world_m_s').max()),'base_max_attitude_drift_rad':float(drift.max()),'base_final_attitude_drift_rad':float(drift[-1]),
      'base_peak_abs_linear_components_m_s':abs(trace['base_linear_velocity_world_m_s']).max(axis=0).tolist(),'base_peak_abs_angular_components_rad_s':abs(trace['base_angular_velocity_world_rad_s']).max(axis=0).tolist(),
      'alignment_angle_rad':alpha,'constraints':checks,'minimum_centerline_distance_m':float(trace['minimum_centerline_distance_m'].min()),'physical_clearance':'NOT_EVALUATED_SOURCE_GEOMETRY_MISSING',
      'end_time_s':float(t[-1]),'samples':len(t),'dt_s':dt,'cpu_s':cpu,'wall_s':wall,'new_physical_attempts':int(kind=='DYNAMICS'),'no_contact':True,
      'rms_position_error_m':float(np.sqrt(np.trapz(norm('position_error_m')**2,t)/max(t[-1],1e-12))),
      'rms_rotation_error_rad':float(np.sqrt(np.trapz(norm('rotation_error_rad')**2,t)/max(t[-1],1e-12))),
      'rms_linear_velocity_error_m_s':float(np.sqrt(np.trapz(norm('linear_velocity_error_m_s')**2,t)/max(t[-1],1e-12))),
      'rms_angular_velocity_error_rad_s':float(np.sqrt(np.trapz(norm('angular_velocity_error_rad_s')**2,t)/max(t[-1],1e-12))),
      'max_hierarchy_linear_residual_m_s':float(trace['hierarchy_linear_residual_m_s'].max()),'max_hierarchy_angular_residual_rad_s':float(trace['hierarchy_angular_residual_rad_s'].max())}
    return Evaluation(metrics,trace)

def check_command(v,q,previous,dt):
    c=assumptions()['controller']
    if not np.isfinite(v).all():return 'NONFINITE_COMMAND'
    if max(abs(v))>c['joint_velocity_limit_rad_s']+1e-9:return 'JOINT_VELOCITY_COMMAND_LIMIT'
    if max(abs(v-previous))/dt>c['joint_acceleration_limit_rad_s2']+1e-8:return 'JOINT_ACCELERATION_COMMAND_LIMIT'
    if max(abs(q[7:]))>c['joint_position_limit_rad']+1e-9:return 'JOINT_POSITION_LIMIT'
    return None

def kinematic(T,psi,dt=None,cpu_limit_s=28800):
    cpu=time.process_time();started=time.perf_counter();k=IndependentSRS();m=registered_model();d=initial(m);q=d.qpos.copy();ref=reference_for(k,q,T,psi)
    source_control=registered_controller('source_nullspace',ControllerSetup({'domain':'paper_compat_srs'},{}),m)
    dt=dt or assumptions()['optimization']['planning_dt_s'];rows=[];t=0.;previous=np.zeros(7);reason=None
    while True:
        try:
            terms=k.terms(q);shape=k.shape(q[7:]);r=ref.sample(t);c=source_control(terms,shape,r)
            reason=check_command(c.velocity,q,previous,dt);v=terms.P@c.velocity
            rows.append(sample(k,q,v,r,t,np.zeros(7),c.velocity,[np.linalg.norm(c.primary_residual[:3]),np.linalg.norm(c.primary_residual[3:])]))
            if reason or t>=T-1e-10:break
            if time.process_time()-cpu>cpu_limit_s:reason='BUDGET_EXHAUSTED';break
            h=min(dt,T-t);mid=integrate_configuration(q,v,h/2);mt=k.terms(mid);ms=k.shape(mid[7:]);mc=source_control(mt,ms,ref.sample(t+h/2))
            q=integrate_configuration(q,mt.P@mc.velocity,h);previous=c.velocity;t=min(T,t+h)
        except (ValueError,np.linalg.LinAlgError) as ex:
            reason=str(ex);break
    if not rows:raise RuntimeError('No valid initial sample: '+str(reason))
    return summarize(rows,T,psi,'KINEMATIC',reason,time.process_time()-cpu,time.perf_counter()-started,dt)

def dynamics(T,psi,method='source_nullspace',dt=None,cpu_limit_s=28800):
    cpu=time.process_time();started=time.perf_counter();a=assumptions()['controller'];dt=dt or a['physics_dt_s'];m=registered_model(dt=dt);d=initial(m);k=IndependentSRS();ref=reference_for(k,d.qpos,T,psi)
    algorithm=registered_controller(method,ControllerSetup({'domain':'paper_compat_srs'},{}),m)
    hqp=algorithm if method=='project_hqp' else None
    qref=d.qpos[7:].copy();dqref=np.zeros(7);ddqref=np.zeros(7);previous=np.zeros(7);command=np.zeros(7);rows=[];inputs=[];steps=[];reason=None;tick=0;hierarchy=[0.,0.]
    task_stride=round(a['task_period_s']/dt)
    while True:
        t=float(d.time);r=ref.sample(min(t,T));mujoco.mj_forward(m,d)
        try:
            if tick%task_stride==0 and t<T-1e-10:
                if hqp:
                    result=hqp.command(d,r);command=result.joint_velocity
                    hierarchy=[result.primary_linear_velocity_residual_m_s,result.primary_angular_velocity_residual_rad_s]
                    if not result.success:reason='HQP_FAILURE:'+result.primary_status+'/'+result.secondary_status
                else:
                    result=algorithm(k.terms(d.qpos),k.shape(d.qpos[7:]),r);command=result.velocity
                    hierarchy=[float(np.linalg.norm(result.primary_residual[:3])),float(np.linalg.norm(result.primary_residual[3:]))]
                reason=reason or check_command(command,d.qpos,previous,a['task_period_s'])
                ddqref=(command-dqref)/a['task_period_s'];previous=command.copy()
            proposal=servo_torque(m,d,qref,dqref,ddqref,{'method':method})
            if not proposal.actuation_valid:reason=proposal.decision
            rows.append(sample(k,d.qpos,d.qvel,r,t,proposal.tau7,command,hierarchy))
            if np.linalg.norm(rows[-1]['position_error_m'])>.10:reason='POSITION_TRACKING_EARLY_STOP'
            if np.linalg.norm(rows[-1]['rotation_error_rad'])>.5:reason='ROTATION_TRACKING_EARLY_STOP'
            if max(abs(d.qvel[6:]))>a['joint_velocity_limit_rad_s']+1e-6:reason='ACTUAL_JOINT_VELOCITY_LIMIT'
            if reason or t>=T-1e-10:break
            if time.process_time()-cpu>cpu_limit_s:reason='BUDGET_EXHAUSTED';break
            h=min(dt,T-t);m.opt.timestep=h;d.ctrl[:]=proposal.tau7;inputs.append(proposal.tau7.copy());steps.append(h)
            mujoco.mj_step(m,d);qref+=h*dqref+.5*h*h*ddqref;dqref+=h*ddqref;tick+=1
        except (ValueError,np.linalg.LinAlgError) as ex:
            reason=str(ex);break
    if not rows:raise RuntimeError('No initial dynamics sample: '+str(reason))
    result=summarize(rows,T,psi,'DYNAMICS',reason,time.process_time()-cpu,time.perf_counter()-started,dt)
    result.trace['applied_ctrl']=np.asarray(inputs).reshape(-1,7);result.trace['integration_dt_s']=np.asarray(steps)
    result.metrics.update(method=method,physics_steps=len(inputs),initializations=1,intermediate_state_injections=0,actuator_channels=m.nu,
      actual_applied_peak_torque_nm=float(np.max(abs(inputs))) if inputs else 0.,torque_trace_scope='proposed pre-step torque; actual applied inputs are saved separately, final/aborted proposals are not applied')
    return result
