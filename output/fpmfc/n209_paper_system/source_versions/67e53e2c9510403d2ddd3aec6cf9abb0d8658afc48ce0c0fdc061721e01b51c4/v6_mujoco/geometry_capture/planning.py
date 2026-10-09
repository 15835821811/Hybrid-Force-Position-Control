"""Kinematic planning only: no torque controller and no mj_step calls."""
import copy
from dataclasses import asdict
import json
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from v6_mujoco.model import default_model_spec,geom_id
from v6_mujoco.collision import build_collision_pairs,signed_distance
from v6_mujoco.fpmfc.controller import FPMFCHQP
from v6_mujoco.fpmfc.run_capture import _controller_config,_constraint_config
from v6_mujoco.fpmfc.target import target_from_config
from v6_mujoco.end_to_end_capture.common import preconfig,read,C1
from v6_mujoco.end_to_end_capture.adapter import initialize
from v6_mujoco.postgrasp.physics import joint_slices
from v6_mujoco.postgrasp_campaign.io import save,save_npz,identity
from .audit import ROOT,siteT,set_body_free,transform
from .design import compile_model,extra_pairs,planning_minimum,config
from .reference import Reference,shape_model

def full_distance_gradient(m,d,p,epsilon=1e-6):
    """Symmetric configuration-tangent FD in isolated copies at one instant."""
    grad=np.zeros(m.nv);base=d.qpos.copy()
    for i in range(m.nv):
        direction=np.zeros(m.nv);direction[i]=1;values=[]
        for sign in [-1,1]:
            c=copy.copy(d);mujoco.mj_integratePos(m,c.qpos,direction,sign*epsilon);mujoco.mj_forward(m,c);values.append(signed_distance(m,c,p))
        grad[i]=(values[1]-values[0])/(2*epsilon)
    assert np.array_equal(base,d.qpos)
    return grad

class GeometryHQP(FPMFCHQP):
    def _clearance_constraints(self,data,mapping):
        rows=[];lower=[];minimum=2.;self.last_geometry=[]
        _,tv=joint_slices(self.model,'target_free_joint')
        drift_velocity=np.zeros(self.model.nv);drift_velocity[tv]=data.qvel[tv]
        for pair in self.pairs:
            dist=signed_distance(self.model,data,pair);minimum=min(minimum,dist)
            if dist>self.config.clearance_activation_m:continue
            if pair.category in ['workspace','satellite','self']:
                dist,g=self.clearance_gradient_for_pair(data,pair,mapping);drift=0.
            else:
                full=full_distance_gradient(self.model,data,pair);g=full@mapping;drift=float(full@drift_velocity)
            minimum_gate=planning_minimum(pair)
            rhs=-self.config.clearance_barrier_gain*(dist-minimum_gate)-drift
            rows.append(g);lower.append(rhs)
            self.last_geometry.append({'pair':pair.name,'distance_m':dist,'gradient':g.tolist(),'target_drift_m_s':drift,'rhs':rhs,'planning_margin_m':minimum_gate})
        return (np.vstack(rows) if rows else np.zeros((0,7)),np.array(lower),minimum)

def make_hqp(m,d):
    cfg=preconfig();shape=shape_model(m,copy.copy(d));hqp=GeometryHQP(default_model_spec(),m,build_collision_pairs(m)+extra_pairs(m),shape,controller_config=_controller_config(cfg),constraint_config=_constraint_config(cfg));return hqp,shape

def target_state(m,d,t):
    s=target_from_config(preconfig()).sample(t);set_body_free(m,d,'target_free_joint',transform(s.center_position_world_m,s.center_rotation_world));_,tv=joint_slices(m,'target_free_joint');d.qvel[tv]=np.r_[s.center_linear_velocity_world_m_s,s.center_rotation_world.T@s.angular_velocity_world_rad_s]

def evaluate(name):
    m=compile_model();d=initialize(m);hqp,shape=make_hqp(m,d);ref=Reference(m,d,name);pairs=build_collision_pairs(m)+extra_pairs(m);qids,vids=default_model_spec().joint_addresses(m)
    rows=[];tasks=[];velocity=np.zeros(7);start=np.zeros(7);all_ok=True;violations=[];dt=.002
    # 2 ms full-horizon grid. Bounds below include a deliberately loose 3 m
    # lever arm for each robot hinge; this exceeds the sum of link offsets
    # and geom bounding radii. No finite-sampling robustness claim is made.
    for step in range(4001):
        t=step*dt;d.time=t;target_state(m,d,t);mujoco.mj_forward(m,d);r=ref.sample(t)
        if step%10==0 and step<4000:
            result=hqp.solve_fpmfc(copy.copy(d),target_position=r.position_world_m,target_velocity=r.linear_velocity_world_m_s,target_rotation=r.rotation_world,target_angular_velocity=r.angular_velocity_world_rad_s,target_arm_angle_rad=r.arm_angle_rad,target_arm_angle_velocity_rad_s=r.arm_angle_velocity_rad_s)
            tasks.append({'time_s':t,**{k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in asdict(result).items()},'geometry_constraints':hqp.last_geometry,'bound_conflicts':hqp.last_bound_conflicts.tolist()})
            start=velocity.copy();velocity=result.joint_velocity.copy()
            if not result.success or np.any(hqp.last_bound_conflicts):all_ok=False;violations.append({'time_s':t,'reason':'hard QP or joint bound failure'})
        distances=np.array([signed_distance(m,d,p) for p in pairs]);F=siteT(m,d,'flange_site')
        row={'time_s':t,'qpos':d.qpos.copy(),'qvel':d.qvel.copy(),'distance_m':distances,'reference_p':r.position_world_m,'actual_p':F[:3,3],
             'position_error_m':np.linalg.norm(F[:3,3]-r.position_world_m),'rotation_error_rad':Rotation.from_matrix(r.rotation_world.T@F[:3,:3]).magnitude(),
             'shape_error_rad':abs((shape.sample(d).angle_rad-r.arm_angle_rad+np.pi)%(2*np.pi)-np.pi)}
        rows.append(row)
        for p,dist in zip(pairs,distances):
            if dist<planning_minimum(p)-1e-9:all_ok=False;violations.append({'time_s':t,'pair':p.name,'distance_m':float(dist),'reason':'planning margin'})
        if step==4000:break
        cmd=start+((step%10)+1)/10*(velocity-start);mapping,_=hqp.reaction_velocity_map(d);dq=mapping@cmd;_,tv=joint_slices(m,'target_free_joint');dq[tv]=d.qvel[tv]
        d.qvel[:]=dq;mujoco.mj_integratePos(m,d.qpos,dq,dt)
    a={k:np.asarray([r[k] for r in rows]) for k in rows[0]};out=ROOT/'planning'/name;save_npz(out/'kinematic_trace.npz',a);save(out/'tasks.json',tasks)
    gates=read(C1/'pairing_manifest.json')['handoff_gates'];terminal={'position':a['position_error_m'][-1]<=gates['position_error_m'],'rotation':a['rotation_error_rad'][-1]<=gates['orientation_error_rad'],'shape':a['shape_error_rad'][-1]<=gates['arm_angle_error_rad']}
    # Conservative per-interval set motion bound for the interpolation defined
    # by the stored generalized velocities. Target rotation is included.
    bounds=[]
    for i in range(4000):
        dq=a['qvel'][i+1];_,bv=joint_slices(m,'base_free_joint');_,tv=joint_slices(m,'target_free_joint')
        robot=dt*(np.linalg.norm(dq[bv][:3])+3*np.linalg.norm(dq[bv][3:])+3*np.sum(abs(dq[vids])))
        target=dt*(np.linalg.norm(dq[tv][:3])+.27*np.linalg.norm(dq[tv][3:]))
        b=np.array([2*robot if p.category in ['tool_robot','self','satellite'] else robot+(target if p.category in ['target_distance','intended_surface'] else 0) for p in pairs])
        bounds.append(a['distance_m'][i]-b-np.array([planning_minimum(p) for p in pairs]))
    envelope_min=np.min(bounds,axis=0)
    qualified=bool(all_ok and all(terminal.values()) and np.all(envelope_min>=0))
    result={'name':name,'qualified':qualified,'scope':'KINEMATIC_PLANNING_ONLY_NOT_REAL_CLOSED_LOOP','new_physics_steps':0,'times_s':[0.,8.], 'samples':4001,
            'all_hard_tasks_passed':all(x['success'] and not any(x['bound_conflicts']) for x in tasks),'min_QP_constraint_slack':min(x['minimum_constraint_slack'] for x in tasks),
            'terminal_gates':{k:bool(v) for k,v in terminal.items()},'terminal_errors':{k:float(a[k][-1]) for k in ['position_error_m','rotation_error_rad','shape_error_rad']},
            'pair_minima':[{'pair':p.name,'distance_m':float(a['distance_m'][:,j].min()),'planning_margin_m':planning_minimum(p),'conservative_step_envelope_slack_m':float(envelope_min[j])} for j,p in enumerate(pairs)],
            'step_envelope':'For piecewise constant generalized-velocity interpolation: hinge lever bound 3 m, robot base radius 3 m, target radius .27 m. Does not bound physical tracking errors or arbitrary pose disturbances.',
            'violations':violations,'files_identity':identity([out/'kinematic_trace.npz',out/'tasks.json'])}
    save(out/'result.json',result);print(json.dumps({k:result[k] for k in ['name','qualified','all_hard_tasks_passed','terminal_errors']},indent=2),flush=True);return result

def run():
    selected=None;results=[]
    for candidate in config()['reference_candidates']:
        result=evaluate(candidate['name']);results.append(result)
        if result['qualified']:selected=candidate['name'];break
    save(ROOT/'trajectory_screening.json',{'candidates':results,'selected':selected,'maximum_candidates':3,'formal_dynamics_attempts':0,'skipped':[x['name'] for x in config()['reference_candidates'] if x['name'] not in [y['name'] for y in results]],'selection_uses_postgrasp_metrics':False})

def refine_saved():
    """Densify the SAME stored kinematic paths where a loose bound is unclear.

    No trajectory generation, controller solves or new candidate evaluation.
    """
    from v6_mujoco.end_to_end_capture.common import arrays
    screening=read(ROOT/'trajectory_screening.json')
    save(ROOT/'trajectory_screening_coarse_bound.json',screening)
    m=compile_model();pairs=build_collision_pairs(m)+extra_pairs(m);_,vids=default_model_spec().joint_addresses(m);_,bv=joint_slices(m,'base_free_joint');_,tv=joint_slices(m,'target_free_joint')
    for result in screening['candidates']:
        name=result['name'];out=ROOT/'planning'/name;save(out/'result_coarse_bound.json',result)
        a=arrays(out/'kinematic_trace.npz');refined=[]
        for j,p in enumerate(pairs):
            if result['pair_minima'][j]['conservative_step_envelope_slack_m']>=0:continue
            worst=2.;queries=0;maxdiv=1
            for i in range(4000):
                dq=a['qvel'][i+1];dt=a['time_s'][i+1]-a['time_s'][i]
                robot=dt*(np.linalg.norm(dq[bv][:3])+3*np.linalg.norm(dq[bv][3:])+3*np.sum(abs(dq[vids])))
                target=dt*(np.linalg.norm(dq[tv][:3])+.27*np.linalg.norm(dq[tv][3:]));bound=2*robot if p.category in ['tool_robot','self','satellite'] else robot+(target if p.category in ['target_distance','intended_surface'] else 0)
                slack=a['distance_m'][i,j]-planning_minimum(p)
                if slack>=bound:worst=min(worst,slack-bound);continue
                divisions=2
                while divisions<=128:
                    sub=[]
                    for u in np.arange(divisions)/divisions:
                        d=mujoco.MjData(m);d.qpos[:]=a['qpos'][i];mujoco.mj_integratePos(m,d.qpos,dq,u*dt);target_state(m,d,a['time_s'][i]+u*dt);mujoco.mj_forward(m,d)
                        sub.append(signed_distance(m,d,p)-planning_minimum(p)-bound/divisions);queries+=1
                    if min(sub)>=0:break
                    divisions*=2
                maxdiv=max(maxdiv,min(divisions,128));worst=min(worst,min(sub))
            result['pair_minima'][j]['refined_step_envelope_slack_m']=float(worst)
            refined.append({'pair':p.name,'distance_queries':queries,'max_subdivisions':maxdiv,'minimum_substep_s':.002/maxdiv,'bounded_clearance_slack_m':float(worst)})
        result['refinement']=refined
        result['qualified']=bool(result['all_hard_tasks_passed'] and all(result['terminal_gates'].values()) and not result['violations'] and all(x.get('refined_step_envelope_slack_m',x['conservative_step_envelope_slack_m'])>=0 for x in result['pair_minima']))
        save(out/'result.json',result)
    screening['selected']=next((x['name'] for x in screening['candidates'] if x['qualified']),None)
    screening['refinement_policy']='Same saved paths, adaptive subdivision only, no change to pose/margins/control/candidate count; initial loose bounds preserved.'
    save(ROOT/'trajectory_screening.json',screening);print(json.dumps({'selected':screening['selected'],'refinement':[x['refinement'] for x in screening['candidates']]},indent=2))

if __name__=='__main__':run()
