"""Affected-contract tests only; total real robot unit dynamics is 12 ms."""
import copy
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from v6_mujoco.model import PROJECT_ROOT,body_id,geom_id,default_model_spec
from v6_mujoco.postgrasp.physics import body_jacobian,momenta,joint_slices,mass_matrix
from v6_mujoco.postgrasp.run import known_wrench_test,read_config
from v6_mujoco.postgrasp_campaign.engine import integration_state
from v6_mujoco.postgrasp_campaign.io import save,read,identity
from .audit import ROOT,ideal,siteT,bodyT,set_body_free,distance,ordered_box_witness
from .adapter import compile_model,initialize,pads,latch_id,extra_pairs
from .reference import Reference,flange_jet
from .geometry import clip_triangle_box
from .planning import full_distance_gradient
from .approach import Approach
from .observer import observe,capture,safety
from .runner import switch,summarize
from .validator import replay
from .physics import momenta

def run():
    findings={};m=compile_model();real=initialize(m);initial=integration_state(m,real).copy();d=ideal(m);d.qvel[:]=0;d.eq_active[latch_id(m)]=True
    for g in pads(m):m.geom_contype[g]=0;m.geom_conaffinity[g]=0
    mujoco.mj_forward(m,d);findings['known_wrench']=known_wrench_test(m,d,read_config())
    # Independent P/H and locked-inertia consistency by rigid virtual twist.
    p=momenta(m,real);v=np.zeros(m.nv);omega=np.array([.13,-.09,.07]);linear=np.array([.02,.04,-.03]);center=np.asarray(p['center_world_m'])
    for name,body in [('base_free_joint','base_link_0'),('target_free_joint','tumbling_target')]:
        _,s=joint_slices(m,name);b=body_id(m,body);R=real.xmat[b].reshape(3,3);v[s]=np.r_[linear+np.cross(omega,real.xpos[b]-center),R.T@omega]
    c=copy.copy(real);c.qvel[:]=v;mujoco.mj_forward(m,c);p2=momenta(m,c)
    np.testing.assert_allclose(p2['linear_momentum_world_kg_m_s'],p['mass_kg']*linear,atol=1e-10)
    np.testing.assert_allclose(p2['angular_momentum_about_center_world_kg_m2_s'],np.asarray(p['locked_inertia_world_kg_m2'])@omega,atol=1e-10)
    np.testing.assert_allclose(.5*v@mass_matrix(m,c)@v,.5*p['mass_kg']*(linear@linear)+.5*omega@np.asarray(p['locked_inertia_world_kg_m2'])@omega,atol=1e-10)
    findings['momentum_and_locked_inertia_virtual_rigid_twist']=True
    findings['dimensions']=[m.nq,m.nv,m.nu];assert (m.nq,m.nv,m.nu)==(21,19,7)
    tool=body_id(m,'n206_rigid_tool');assert m.body_mass[tool]>0 and np.all(m.body_inertia[tool]>0)
    # Triangle crossing without an inside vertex (vertex-only regression).
    tri=np.array([[-2,0,0],[2,0,0],[0,2,0.]])
    assert not np.any(np.all(abs(tri)<.5,axis=1)) and len(clip_triangle_box(tri,np.ones(3)*.5))>=3
    findings['triangle_face_crossing_without_inside_vertices']=True
    # Fixed-chain invariance is already measured using saved real states.
    audit=read(ROOT/'geometry_audit.json');assert audit['tests']['fixed_chain_error']<1e-12 and audit['tests']['mesh_world_max_error_m']<1e-7
    findings['distance_and_fixed_transform_tests']=audit['tests']
    ref=Reference(m,real);pT,RT,vT,wT,aT,alphaT=flange_jet(m,real,8);r=ref.sample(8)
    for a,b in [(r.position_world_m,pT),(r.rotation_world,RT),(r.linear_velocity_world_m_s,vT),(r.angular_velocity_world_rad_s,wT),(r.linear_acceleration_world_m_s2,aT),(r.angular_acceleration_world_rad_s2,alphaT)]:np.testing.assert_allclose(a,b,atol=1e-12)
    eps=1e-4;pminus,Rminus,vminus,*_=flange_jet(m,real,8-eps);pplus,Rplus,vplus,*_=flange_jet(m,real,8+eps)
    errs={'velocity_fd_m_s':float(np.linalg.norm((pplus-pminus)/(2*eps)-vT)),'acceleration_fd_m_s2':float(np.linalg.norm((vplus-vminus)/(2*eps)-aT)), 'omega_fd_rad_s':float(np.linalg.norm(Rotation.from_matrix(Rplus@Rminus.T).as_rotvec()/(2*eps)-wT))}
    assert max(errs.values())<1e-8;findings['boundary_jet_derivatives']=errs
    for name in ['standard_C1','normal_approach_40mm','normal_approach_80mm']:
        rr=Reference(m,real,name);left=rr.sample(8-1e-6);end=rr.sample(8)
        assert np.linalg.norm(left.linear_acceleration_world_m_s2-end.linear_acceleration_world_m_s2)<2e-5
    # Full target tangent gradient checked independently with moving-point
    # Jacobians, including angular components, plus a directional FD test.
    drift=[];source=np.load(ROOT/'planning/standard_C1/kinematic_trace.npz')
    c=copy.copy(real);c.qpos[:]=source['qpos'][3750];c.qvel[:]=source['qvel'][3750];mujoco.mj_forward(m,c);_,tv=joint_slices(m,'target_free_joint')
    for pair in [p for p in extra_pairs(m) if p.category=='target_distance']:
        full=full_distance_gradient(m,c,pair);segment=np.zeros(6);dist=mujoco.mj_geomDistance(m,c,pair.geom_a,pair.geom_b,2,segment)
        segment,reversed_,_=ordered_box_witness(m,c,mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,pair.geom_b),segment);normal=(segment[3:]-segment[:3])/dist
        jb=body_jacobian(m,c,int(m.geom_bodyid[pair.geom_b]),segment[3:]);analytic=normal@jb[:3,tv]@c.qvel[tv];fd=float(full[tv]@c.qvel[tv]);drift.append({'pair':pair.name,'target_drift_fd_m_s':fd,'target_point_jacobian_m_s':float(analytic),'error':float(abs(fd-analytic))})
    save(ROOT/'distance_drift_diagnostic.json',{'rows':drift})
    assert max(x['error'] for x in drift)<2e-6;findings['target_drift_including_rotation']=drift
    # Finite perturbation coverage, not a universal robustness proof.
    minclear=2.;samples=0;ideal_state=ideal(m);B=bodyT(m,ideal_state,'base_link_0');G=siteT(m,ideal_state,'target_grasp_site');nonintended=[p for p in extra_pairs(m) if p.category=='target_distance']
    for axis in range(3):
        for sign in [-1,1]:
            for translation,angle in [(.0001,np.deg2rad(.05)),(.0005,np.deg2rad(.1))]:
                c=copy.copy(ideal_state);R=Rotation.from_rotvec(np.eye(3)[axis]*sign*angle).as_matrix();H=np.eye(4);H[:3,:3]=R;H[:3,3]=G[:3,3]-R@G[:3,3]+np.eye(3)[axis]*sign*translation;set_body_free(m,c,'base_free_joint',H@B);mujoco.mj_forward(m,c)
                minclear=min(minclear,min(float(mujoco.mj_geomDistance(m,c,p.geom_a,p.geom_b,2,None)) for p in nonintended));samples+=1
    assert minclear>.002;findings['terminal_perturbations']={'samples':samples,'minimum_target_clearance_m':minclear,'scope':'axis perturbations at capture tolerance and holding deformation only; finite samples do not cover all combined disturbances','distance_numeric_allowance_m':1e-5}
    assert np.array_equal(initial,integration_state(m,real));findings['isolated_static_operations_preserve_main_data']=True
    # At exactly t=8 invalid actual geometry must reject the event.
    m=compile_model();c=initialize(m);c.time=8;approach=Approach(m,c);row=observe(m,c,approach.trajectory,approach.shape);assert not capture(row)
    try:switch(m,c,row)
    except RuntimeError:pass
    else:raise AssertionError('capture failure accepted')
    assert not c.eq_active[latch_id(m)];findings['actual_t8_bad_state_rejects_latch']=True
    ticks=[];steps=0
    for scenario in ['nominal','fine']:
        m=compile_model(scenario);d=initialize(m);ctx=Approach(m,d);initial={'integration_state':integration_state(m,d).tolist(),'model_masks':np.array([m.geom_contype,m.geom_conaffinity]).tolist()};rows=[]
        for i in range(round(.006/m.opt.timestep)+1):
            before=integration_state(m,d)
            if i%round(.002/m.opt.timestep)==0:
                tau=ctx.update(d,i==0);assert np.array_equal(before,integration_state(m,d));d.ctrl[:]=tau
            row=observe(m,d,ctx.trajectory,ctx.shape);rows.append(row)
            if i<round(.006/m.opt.timestep):mujoco.mj_step(m,d);steps+=1
        a={k:np.array([r[k] for r in rows]) for k in rows[0]};v=replay(compile_model(scenario),initial,a);assert v['passed'] and summarize(a,'COMPLETED')['performance']=='NOT_EVALUATED'
        ticks.append({'scenario':scenario,'duration_s':.006,'replay':v})
    findings['short_isolation_clock_and_replay']=ticks
    save(ROOT/'unit_tests.json',{'passed':True,'real_robot_unit_steps':steps,'max_unit_duration_s':.006,'new_formal_attempts':0,'findings':findings,'implementation_identity':identity([__file__])})
    ledger=read(ROOT/'run_ledger.json');ledger['unit_tests'].append({'passed':True,'real_robot_steps':steps,'unit_duration_each_s':.006,'replay_steps':sum(x['replay']['steps'] for x in ticks)});save(ROOT/'run_ledger.json',ledger)
    print({'unit_tests':'passed','physics_steps':steps})

if __name__=='__main__':run()
