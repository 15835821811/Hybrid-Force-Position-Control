"""Bounded mathematical and interface checks, no full robot trajectories."""
import copy
import io
import time
import unittest
from dataclasses import replace
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from ..estimation.filter import AccelerationSnapshot,predict_snapshot
from ..estimation.tests import suite as estimator_suite
from ..estimation.controller import AccelerationController
from ..common import clean
from .contracts import *
from .parameterization import PlannedReference
from .trajectory_segment import JetSegment

def default_contract():
    return dict(version='V1',dimensions=5,theta=['remaining_time_s','normal_bump_m','tangent_bump_m','psi_mid_correction_rad','psi_end_correction_rad'],
        minimum_remaining_s=.6,offset_m=.02,max_offset_m=.04,shape_rad=.25,max_shape_correction_rad=.6,
        max_shape_velocity_rad_s=.6,max_shape_accel_rad_s2=1.2,max_virtual_rate=2.,max_virtual_accel=2.,
        planning_period_s=.8,prediction_horizon_s=1.2,screen_horizon_s=2.4,dynamic_survivors=2,candidates_per_update=5,
        cache_domain=dict(q_rad=.02,dq_rad_s=.15,p_m=.003,R_rad=.03),
        terminal='Q_GE(1)=I from frozen sites; no fitted target or tool transform',
        timing='u=8s; normalized s=u/8, derivatives explicitly transformed; legacy 6/7.8 phases correspond to s=.75/.975, not physical seconds',
        C2='quintic from executed reference jets, sixth degree endpoint-zero bump; analytic SO(3) physical omega/alpha',
        time_domain='arrival <=19.9 s; global approach deadline remains20s',
        range_basis='20mm increment equals frozen tool length and half existing40mm robot clearance; total correction cap40mm. Shape correction .25rad is conservative relative to joint ranges; .6rad cap and explicit velocity/acceleration bounds. None constitutes guaranteed reachability.',
        horizon_basis='1.2s covers .8s scheduling plus .4s transition lead; preflight reports measured joint braking estimates. Near arrival horizon truncates to max(.2,remaining+.06) because finite local reference ends; no all-horizon claim.',
        scores='kinematic: 10*negative_margin/10mm +2*speed_excess +tracking/8mm +.05*base_omega/.02rad_s +.05 change penalty. Hard dynamic checks first; no scalar success claim from score.',
        rejection_limits='unchanged HQP/physical safety; existing 8mm tracking and .05m/s/.1rad/s residual are performance rejection, not collision',
        sensitivity_thresholds=dict(reference_m=1e-7,action_rad_s=1e-6,margin_m=1e-7),
        isolation='independent prior dynamics and copied command/reference histories; no future packet or truth parameters',
        real_time='NON_REALTIME_SIMULATION',validity='valid_until dynamic horizon; next update .8s or cache-domain exit; expired/no accepted action SIM_ABORT')

def make_ref():
    c=read(S02/'runs/E0_C2/config.json');owner=AccelerationController(c['prior'],c['mission'])
    r=PlannedReference(owner.model,owner.data,c['mission']);P=np.eye(18)*1e-10
    n=r.nominal.target.sample(.04)
    e=AccelerationSnapshot(.04,.04,n.center_position_world_m,n.center_rotation_world,n.center_linear_velocity_world_m_s,n.angular_velocity_world_rad_s,P[:12,:12],True,np.zeros(6),np.array([.003,-.002,.001]),np.array([.001,.002,-.001]),P,False,.0013536209515421882,.016185201664205662,10.,.002)
    r.estimate=e
    return owner,r,e

class ReferenceTests(unittest.TestCase):
    def test_atomic_sharing_retry_retains_same_value_and_is_bounded(self):
        from unittest.mock import patch
        import tempfile
        from . import contracts
        with tempfile.TemporaryDirectory() as td:
            path=Path(td)/'budget.json';value={'evaluations':442,'retained':True};calls=[]
            real=contracts._save
            def flaky(p,v):
                calls.append(copy.deepcopy(v))
                if len(calls)<3:raise PermissionError('injected Windows sharing lock')
                return real(p,v)
            with patch.object(contracts,'_save',side_effect=flaky),patch.object(contracts.time,'sleep'):
                contracts.save(path,value)
            self.assertEqual(read(path),value);self.assertTrue(all(x==value for x in calls))
            with patch.object(contracts,'_save',side_effect=PermissionError('persistent')) as fail,patch.object(contracts.time,'sleep'):
                with self.assertRaises(PermissionError):contracts.save(path,{'evaluations':443})
                self.assertEqual(fail.call_count,10)
            self.assertEqual(read(path),value)
    def test_quintic_and_bump_jets(self):
        p=JetSegment.connect(2.,3.,[.2,.3,-.4],[.5,-.2,.1],bump=.6)
        np.testing.assert_allclose(p.jet(2),[.2,.3,-.4],atol=1e-12)
        np.testing.assert_allclose(p.jet(5),[.5,-.2,.1],atol=1e-12)
    def test_fixed_baseline_and_tool(self):
        c,r,e=make_ref();r=r.candidate(.04,[8,0,0,0,0]);legacy=copy.deepcopy(c.reference);legacy.estimate=e
        for x in [0,1,3,5,7,8]:
            legacy.progress=x;legacy.rate=1.;legacy.accel=0.;r.clock=x+.04;r._sync()
            a,b=r.sample(0),legacy.sample(0)
            for k in ['position_world_m','rotation_world','linear_velocity_world_m_s','angular_velocity_world_rad_s','arm_angle_rad','arm_angle_velocity_rad_s']:np.testing.assert_allclose(getattr(a,k),getattr(b,k),atol=2e-12)
        T=np.eye(4);T[:3,:3]=a.rotation_world;T[:3,3]=a.position_world_m
        G=np.eye(4);G[:3,:3]=e.R;G[:3,3]=e.p
        np.testing.assert_allclose(T@r.T_FE,G@r.T_OG,atol=1e-12)
    def test_independent_derivatives_and_transport(self):
        _,r,e=make_ref();r=r.candidate(.04,[9,.02,-.015,.2,-.1]);t=3.1;h=1e-4
        def sample(x):
            f=copy.deepcopy(r);f.shaped=predict_snapshot(e,x);return f.sample_at(x)
        a,b,c=sample(t-h),sample(t),sample(t+h)
        np.testing.assert_allclose((c.position_world_m-a.position_world_m)/(2*h),b.linear_velocity_world_m_s,atol=2e-8)
        np.testing.assert_allclose((c.linear_velocity_world_m_s-a.linear_velocity_world_m_s)/(2*h),b.linear_acceleration_world_m_s2,atol=2e-8)
        w=Rotation.from_matrix(c.rotation_world@a.rotation_world.T).as_rotvec()/(2*h)
        np.testing.assert_allclose(w,b.angular_velocity_world_rad_s,atol=2e-8)
        np.testing.assert_allclose((c.angular_velocity_world_rad_s-a.angular_velocity_world_rad_s)/(2*h),b.angular_acceleration_world_rad_s2,atol=2e-8)
        np.testing.assert_allclose((c.arm_angle_velocity_rad_s-a.arm_angle_velocity_rad_s)/(2*h),b.arm_angle_acceleration_rad_s2,atol=2e-8)
        end=copy.deepcopy(r);end.clock=end.arrival;end._sync();sample=end.sample(0)
        self.assertGreater(np.linalg.norm(sample.linear_velocity_world_m_s),.001)
        self.assertEqual(end.normalized_progress,1.)
    def test_switch_C2_and_unwrapped_shape(self):
        _,r,e=make_ref();r=r.candidate(.04,[8,.02,-.02,.15,.2]);r.clock=2.4;r._sync();a=r.sample_at(2.4)
        z=r.candidate(2.4,[6.,-.02,.01,-.15,-.2]);b=z.sample_at(2.4)
        for key in a.__dataclass_fields__:
            np.testing.assert_allclose(getattr(a,key),getattr(b,key),atol=3e-12)
        angles=[z.sample_at(t).arm_angle_rad for t in np.linspace(2.4,8.4,301)]
        self.assertLess(max(abs(np.diff(angles))),.05)
    def test_named_gradient_at_same_state(self):
        from v6_mujoco.geometry_capture.planning import full_distance_gradient
        from v6_mujoco.collision import signed_distance
        c,r,e=make_ref();d=c.data;v=np.linspace(-.02,.02,c.model.nv);eps=2e-6
        errors=[]
        for p in c.hqp.pairs[-4:]:
            g=full_distance_gradient(c.model,d,p);plus=copy.copy(d);minus=copy.copy(d)
            mujoco.mj_integratePos(c.model,plus.qpos,v,eps);mujoco.mj_integratePos(c.model,minus.qpos,v,-eps)
            mujoco.mj_forward(c.model,plus);mujoco.mj_forward(c.model,minus)
            delta=(signed_distance(c.model,plus,p)-signed_distance(c.model,minus,p))/(2*eps)
            errors.append(abs(delta-g@v))
        self.assertLess(max(errors),1e-6)
    def test_no_truth_and_expiry(self):
        from .planner import PlanningController,PlanningAdapter
        from ..contracts import ControllerSetup
        cfg=read(S02/'runs/E0_C2/config.json');cfg['mission']['s03']=default_contract();cfg['mission']['s03_method']='B2_time_path_shape'
        setup=ControllerSetup(cfg['prior'],cfg['mission']);self.assertFalse(hasattr(setup,'truth_evaluation_only'))
        c=PlanningController(cfg['prior'],cfg['mission']);c.reference.valid_until=.1
        _,r,e=make_ref();c.reference=r.candidate(.04,[8,0,0,0,0]);c.reference.valid_until=.1
        class P:time=.12
        with self.assertRaisesRegex(RuntimeError,'NO_VERIFIED_CONTROL'):c.update(P(),True)
        self.assertEqual(c.predictor.failure,'PLAN_EXPIRED')

def run_tests():
    OUT.mkdir(parents=True,exist_ok=True)
    if not (OUT/'planning_contract.json').exists():save(OUT/'planning_contract.json',default_contract())
    cpu,wall=time.process_time(),time.perf_counter();stream=io.StringIO()
    suite=unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromTestCase(ReferenceTests),estimator_suite()])
    result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    (OUT/'module_tests.log').write_text(stream.getvalue(),encoding='utf-8');print(stream.getvalue(),flush=True)
    out=dict(passed=result.wasSuccessful(),count=result.testsRun,failures=len(result.failures),errors=len(result.errors),new_plant_steps=0,
        scope='bounded synthetic/reference/interface/CA18/persistence; candidate authority and B0 equivalence are separate gates')
    save(OUT/'module_tests.json',out);charge('module_tests',cpu,wall,**out);return out
