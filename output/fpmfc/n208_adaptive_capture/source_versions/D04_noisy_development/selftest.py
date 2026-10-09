"""Independent analytic, isolated-body and static-interface tests; no robot rollout."""
import copy
import dataclasses
import json
import time
import traceback
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from .common import ROOT,config,save,init_ledger
from .contracts import SensorPacket,PoseObservation,StateEstimate
from .momentum_regressor import parameters,regressor,pseudoinertia,physical,state_jacobian
from .inertial_estimator import InertialEstimator
from .state_estimator import StateEstimator
from .plant import Plant,nominal_truth,supported_truth
from .sensors import SensorFrontend
from .known_model import compile_known,robot_momentum
from .controller import Controller
from .relative_reference import RelativeReference
from v6_mujoco.postgrasp_campaign.engine import integration_state

def momentum_tests():
    rng=np.random.default_rng(208);mass=23.;c=np.array([.01,-.015,.005]);A=Rotation.from_rotvec([.3,-.2,.4]).as_matrix();Ic=A@np.diag([.2,.25,.31])@A.T;pi=parameters(mass,c,Ic)
    Ys=[];hs=[];maxerr=0.
    for i in range(50):
        p=rng.normal(size=3);R=Rotation.random(random_state=rng).as_matrix();v=rng.normal(size=3)*.1;w=rng.normal(size=3)*.2
        pc=p+R@c;vc=v+np.cross(w,R@c);P=mass*vc;h=np.r_[P,R@Ic@R.T@w+np.cross(pc,P)]
        Y=regressor(p,R,v,w);maxerr=max(maxerr,np.max(abs(Y@pi-h)));Ys.append(Y);hs.append(h)
        O=np.array([.5,-.2,.1]);moved=regressor(p,R,v,w,O)@pi
        assert np.allclose(moved,np.r_[P,h[3:]-np.cross(O,P)],atol=1e-13)
    assert maxerr<1e-13
    prior=config('target_prior');cfg=config();DY=np.diff(Ys,axis=0).reshape(-1,10);dh=np.diff(hs,axis=0).reshape(-1)
    estimators=[]
    for m in [15.,30.]:
        pr=copy.deepcopy(prior);pr['mass_kg']=m
        est=InertialEstimator(pr,cfg);est.solve(DY/1e-4,dh/1e-4)
        assert est.rank==10 and est.accepted and np.linalg.norm(est.pi-pi)<1e-6,est.snapshot()
        assert physical(est.pi,prior);estimators.append(est.snapshot())
    # Free constant principal-axis spin: no mass information and homogeneous
    # inertia equation. Its scalar multiple remains indistinguishable.
    Yfree=[regressor(np.array([1.,.5,.3]),Rotation.from_rotvec([0,0,.1*t]).as_matrix(),np.zeros(3),np.array([0,0,.1])) for t in np.linspace(0,10,50)]
    Af=np.diff(Yfree,axis=0).reshape(-1,10);pif=parameters(20,np.zeros(3),.3*np.eye(3))
    assert np.linalg.norm(Af@pif)<1e-12 and np.linalg.norm(Af@(2*pif))<1e-12 and np.all(Af[:,0]==0)
    free=InertialEstimator(prior,cfg);free.solve(Af/1e-4,np.zeros(len(Af)));assert free.rank<10
    return {'direct_com_momentum_max_error':float(maxerr),'full_excitation_prior_comparison':estimators,'degenerate':free.snapshot(),'absolute_scale_unobservable':True,'scope':'independent random kinematic momentum data; not a task or dynamic excitation claim'}

def isolated_free(scale=1.,mass_only=False):
    mass=20*scale;I=np.array([.2,.3,.4])*(1 if mass_only else scale)
    xml=f'<mujoco><option timestep="0.001" gravity="0 0 0" integrator="RK4"/><worldbody><body><freejoint/><inertial mass="{mass}" pos=".01 -.012 .004" diaginertia="{I[0]} {I[1]} {I[2]}"/><geom type="box" size=".15 .15 .15" contype="0" conaffinity="0"/></body></worldbody></mujoco>'
    m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);d.qvel[:]=[.02,-.01,.03,.13,-.17,.21];mujoco.mj_forward(m,d)
    traces=[];momentum=[];energy=[]
    for i in range(2001):
        s=copy.copy(d);mujoco.mj_forward(m,s);R=s.xmat[1].reshape(3,3);w=R@s.qvel[3:];rc=R@m.body_ipos[1];v=s.qvel[:3]+np.cross(w,rc);P=mass*v;H=R@np.diag(I)@R.T@w+np.cross(s.xipos[1],P)
        momentum.append(np.r_[P,H]);energy.append(.5*mass*(v@v)+.5*w@(R@np.diag(I)@R.T)@w);traces.append(np.r_[d.qpos,d.qvel])
        if i<2000:mujoco.mj_step(m,d)
    a=np.array(traces);h=np.array(momentum);err=np.max(abs(h-h[0]),axis=0)
    assert np.max(err)<1e-9 and np.ptp(energy)<1e-10
    assert np.linalg.norm(a[-1,-3:]-a[0,-3:])>.001
    return a,{'momentum_component_drift':err.tolist(),'energy_drift':float(np.ptp(energy)),'steps':2000}

def physics_tests():
    a,metrics=isolated_free();scaled,_=isolated_free(1.6);mass,_=isolated_free(1.6,True)
    # At nonzero COM, changing mass alone changes coupling only through a fixed
    # COM velocity; use uniform scaling for exact whole-body trajectory test.
    assert np.max(abs(a-scaled))<1e-10 and np.max(abs(a-mass))<1e-10
    return {'free':metrics,'uniform_scale_state_error':float(np.max(abs(a-scaled))),'isolated_steps':6000,'mass_only_state_error':float(np.max(abs(a-mass)))}

def contact_tests():
    peaks=[];histories=[];errors=[]
    for speed in [.02,.08]:
        xml='<mujoco><option timestep=".0005" gravity="0 0 0" integrator="RK4"/><worldbody><body pos="-.1005 0 0"><freejoint/><inertial pos="0 0 0" mass="2" diaginertia=".01 .01 .01"/><geom type="sphere" size=".05" solref=".01 1" margin="0"/></body><body><freejoint/><inertial pos="0 0 0" mass="20" diaginertia=".3 .3 .3"/><geom type="sphere" size=".05" solref=".01 1" margin="0"/></body></worldbody></mujoco>'
        m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);d.qvel[0]=speed;fs=[];es=[]
        for i in range(600):
            s=copy.copy(d);mujoco.mj_forward(m,s)
            F=s.qfrc_constraint[6:9];F2=s.qfrc_constraint[:3];es.append(np.linalg.norm(F+F2));fs.append(np.linalg.norm(F))
            # Centered sphere translations: force power and Newton balance.
            assert np.linalg.norm(20*s.qacc[6:9]-F)<1e-9
            assert abs(F@s.qvel[6:9]+F2@s.qvel[:3]-s.qfrc_constraint@s.qvel)<1e-10
            mujoco.mj_step(m,d)
        peaks.append(max(fs));histories.append(fs);errors.append(max(es))
    assert peaks[1]>2*peaks[0] and max(errors)<1e-10
    return {'speed_m_s':[.02,.08],'force_peaks_n':peaks,'action_reaction_errors_n':errors,'steps':1200,'scope':'isolated tool-equivalent / target contact, no force curve injection; full robot speed ablation not claimed'}

def state_tests():
    cfg=config();f=StateEstimator(cfg);p0=np.array([.8,.1,.9]);v=np.array([.01,-.02,.03]);w=np.array([.07,-.02,.03]);last=None;maxerr=0.
    for k in range(501):
        t=k*.002;z=PoseObservation(t,p0+v*t,Rotation.from_rotvec(w*t).as_matrix(),np.eye(6)*1e-18)
        packet=SensorPacket(t+.012,np.r_[np.zeros(3),1.,np.zeros(3)],np.zeros(6),np.zeros(7),np.zeros(7),np.zeros(7),np.zeros(6),False,(z,))
        e=f.update(packet)
        if k>10:maxerr=max(maxerr,np.linalg.norm(e.p-(p0+v*(t+.012))),np.linalg.norm(e.w-w))
        assert e.measurement_time<=packet.time
    assert maxerr<1e-7,maxerr
    return {'delayed_constant_twist_max_error':float(maxerr),'prediction_delay_s':.012}

def isolation_tests():
    truth=nominal_truth();a=Plant(truth);b=Plant(dataclasses.replace(truth,mass=31.))
    ca=Controller(config('target_prior'),config());cb=Controller(config('target_prior'),config());sensor=SensorFrontend(config('sensors')['ideal'])
    initial=integration_state(a.model,a.data).copy();outputs=[]
    for i in range(31):
        # Static sensor-input replay fixture, no mj_step / no task rollout.
        scratch=copy.copy(a.data);scratch.time=i*.002;packet=sensor.sample(a.model,scratch)
        ta,la=ca.update(packet,i%10==0);tb,lb=cb.update(packet,i%10==0)
        assert np.array_equal(ta,tb) and la==lb;outputs.append(ta)
    assert np.array_equal(initial,integration_state(a.model,a.data))
    sb=SensorFrontend(config('sensors')['ideal']);pa=SensorFrontend(config('sensors')['ideal']).sample(a.model,a.data);pb=sb.sample(b.model,b.data)
    assert np.array_equal(pa.poses[0].position,pb.poses[0].position) and np.array_equal(pa.poses[0].rotation,pb.poses[0].rotation)
    assert ca.model is not a.model and cb.model is not b.model
    return {'identical_packets_identical_output':True,'mass_change_same_initial_pose_sensor':True,'sensor_preserves_integration_state':True,'controller_model_is_independent':True,'static_controller_calls':62,'robot_physics_steps':0}

def reference_tests():
    m,d=compile_known(config('target_prior'));ref=RelativeReference(m,d);errs=[]
    for t in [0.,2.,6.,7.9,8.]:
        n=ref.nominal.target.sample(t);ref.progress=t;ref.estimate=StateEstimate(t,t,n.center_position_world_m,n.center_rotation_world,n.center_linear_velocity_world_m_s,n.angular_velocity_world_rad_s,np.eye(12)*1e-18,True,np.zeros(6));r=ref.sample(t);old=ref.nominal.sample(t)
        errs.append(np.linalg.norm(r.position_world_m-old.position_world_m)+np.linalg.norm(r.linear_velocity_world_m_s-old.linear_velocity_world_m_s)+np.linalg.norm(r.angular_velocity_world_rad_s-old.angular_velocity_world_rad_s))
    assert max(errs)<1e-12,errs
    e=ref.estimate;ref.estimate=dataclasses.replace(e,p=e.p+np.array([.001,0,0]));shift=ref.sample(8).position_world_m-r.position_world_m
    assert np.allclose(shift,[.001,0,0],atol=1e-12)
    return {'nominal_reduction_max_error':float(max(errs)),'measured_target_shift_reference_shift':shift.tolist()}

def derivative_tests():
    p=np.array([.8,.1,.9]);R=Rotation.from_rotvec([.2,-.3,.1]).as_matrix();v=np.array([.02,.03,-.01]);w=np.array([.04,-.1,.05]);pi=parameters(23,[.01,-.015,.004],np.diag([.2,.3,.35]));eps=1e-6;G=state_jacobian(p,R,v,w,pi);num=np.zeros((6,12))
    for i in range(12):
        dx=np.eye(12)[i]*eps;values=[]
        for s in [-1,1]:values.append(regressor(p+s*dx[:3],Rotation.from_rotvec(s*dx[3:6]).as_matrix()@R,v+s*dx[6:9],w+s*dx[9:])@pi)
        num[:,i]=(values[1]-values[0])/(2*eps)
    assert np.max(abs(num-G))<1e-8
    m,d=compile_known(config('target_prior'));ref=RelativeReference(m,d);u=6.4;rate=.6;acc=.1
    def sample(t):
        ref.progress=u+rate*t+.5*acc*t*t;ref.rate=rate+acc*t;ref.accel=acc;ref.estimate=StateEstimate(t,t,p+v*t,Rotation.from_rotvec(w*t).as_matrix()@R,v,w,np.eye(12)*1e-12,True,np.zeros(6));return ref.sample(t)
    l=sample(-eps);r=sample(eps);c=sample(0.)
    dv=np.linalg.norm((r.position_world_m-l.position_world_m)/(2*eps)-c.linear_velocity_world_m_s)
    da=np.linalg.norm((r.linear_velocity_world_m_s-l.linear_velocity_world_m_s)/(2*eps)-c.linear_acceleration_world_m_s2)
    dw=np.linalg.norm(Rotation.from_matrix(r.rotation_world@l.rotation_world.T).as_rotvec()/(2*eps)-c.angular_velocity_world_rad_s)
    dalpha=np.linalg.norm((r.angular_velocity_world_rad_s-l.angular_velocity_world_rad_s)/(2*eps)-c.angular_acceleration_world_rad_s2)
    assert max(dv,da,dw,dalpha)<1e-7,(dv,da,dw,dalpha)
    return {'momentum_state_jacobian_error':float(np.max(abs(num-G))),'reference_derivative_errors':[dv,da,dw,dalpha]}

def prediction_tests():
    from .governor import BrakeGovernor
    from .controller import DAMPING
    from .information_gate import InformationGate
    from v6_mujoco.model import site_id,default_model_spec
    prior=config('target_prior');m,d=compile_known(prior);spec=default_model_spec();it=site_id(m,'postgrasp_tool_interface');ig=site_id(m,'target_grasp_site');q=m.site_quat[ig];Rlocal=Rotation.from_quat(np.r_[q[1:],q[0]]).as_matrix();R=d.site_xmat[it].reshape(3,3)@Rlocal.T;p=d.site_xpos[it]-R@m.site_pos[ig]
    bq,bv=spec.base_slices(m);qj,vj=spec.joint_addresses(m);packet=SensorPacket(0.,d.qpos[bq].copy(),np.zeros(6),d.qpos[qj].copy(),np.zeros(7),np.zeros(7),np.zeros(6),True,())
    e=StateEstimate(0.,0.,p,R,np.zeros(3),np.zeros(3),np.eye(12)*1e-18,True,np.zeros(6));gov=BrakeGovernor(prior,DAMPING);pi=parameters(prior['mass_kg'],prior['com_m'],prior['inertia_com_kg_m2'])
    assert gov.choose(packet,e,pi) and gov.alpha==1.
    safe_prediction=gov.log[-1]
    assert not gov.choose(packet,dataclasses.replace(e,v=np.array([1.,0,0])),pi)
    # Independent predictive fixture, not a real-plant rollout: audit an
    # energized mated system with nonzero COM and a rotated inertia tensor.
    from .physical_audit import inspect
    from .governor import put_inertia
    from .known_model import set_measured
    pm,pd=gov.models[0];A=Rotation.from_rotvec([.2,-.1,.3]).as_matrix()
    put_inertia(pm,parameters(23,[.01,-.008,.004],A@np.diag([.2,.25,.3])@A.T))
    set_measured(pm,pd,packet,dataclasses.replace(e,v=np.array([.001,-.001,.002]),w=np.array([.01,-.02,.03])))
    pd.eq_active[0]=True;pd.ctrl[:]=np.linspace(-.1,.1,7);mujoco.mj_forward(pm,pd);physical=inspect(pm,pd)
    for key,limit in [('interface_generalized_reconstruction_error',1e-8),('interface_power_residual_w',1e-8),('target_newton_residual_n',1e-6),('target_euler_residual_nm',1e-7)]:assert np.linalg.norm(physical[key])<limit,(key,physical[key])
    est=InertialEstimator(prior,config());est.pi[0]=25;est.rank=3;est.accepted=True;est.absolute_excitation=True;est.predict=[{'rank':3,'posterior':1.,'prior':3.} for _ in range(5)]
    gate=InformationGate(prior);new,used=gate.update(est,.02,True,True);assert used and np.linalg.norm((new-est.pi0)/est.scales)<=.001+1e-12
    fallback,used=gate.update(est,.02,False,True);assert not used and np.linalg.norm(fallback-est.pi0)<1e-12
    return {'static_mated_prediction':safe_prediction,'unsafe_prediction_rejected':True,'parameter_rate_bound_verified':True,'physical_fixture_residuals':{k:np.asarray(v).tolist() for k,v in physical.items()},'robot_plant_steps':0,'predictive_model_steps_upper_bound':120}

def progress_tests():
    m,d=compile_known(config('target_prior'));r=RelativeReference(m,d);dt=.002;rates=[];accels=[];jerks=[]
    # Alternating demands plus a long pause test both boundaries. No plant step.
    for k in range(12000):
        r.advance(dt,allow=bool(k<200 or (500<k<1000) or k>9000));rates.append(r.rate);accels.append(r.accel);jerks.append(r.jerk)
    assert min(rates)>=-1e-12 and max(rates)<=1+1e-12 and max(abs(np.array(accels)))<=.5+1e-12 and max(abs(np.array(jerks)))<=1+1e-12
    derivative_error=np.max(abs(np.diff(rates)/dt-(np.array(accels[1:])+accels[:-1])*.5))
    assert derivative_error<1e-6
    return {'rate_range':[min(rates),max(rates)],'max_acceleration':float(max(abs(np.array(accels)))),'max_jerk':float(max(abs(np.array(jerks)))),'rate_derivative_consistency':float(derivative_error)}

def run():
    ledger=init_ledger();started=time.time();cpu=time.process_time();results={};failed=False
    for fn in [momentum_tests,physics_tests,contact_tests,state_tests,isolation_tests,reference_tests,derivative_tests,prediction_tests,progress_tests]:
        try:results[fn.__name__]={'passed':True,'result':fn()}
        except Exception:results[fn.__name__]={'passed':False,'error':traceback.format_exc()};failed=True
        print(fn.__name__,json.dumps(results[fn.__name__]),flush=True)
    stamp=time.strftime('%Y%m%d_%H%M%S',time.gmtime());path=ROOT/('selftests/'+stamp+'.json');save(path,results)
    ledger=init_ledger();ledger['cpu_s']+=time.process_time()-cpu;ledger['isolated_tests'].append({'path':str(path.relative_to(ROOT)),'passed':not failed,'started_epoch':started,'finished_epoch':time.time(),'robot_physics_steps':0});save(ROOT/'run_ledger.json',ledger)
    if failed:raise SystemExit(1)
    return results

if __name__=='__main__':run()
