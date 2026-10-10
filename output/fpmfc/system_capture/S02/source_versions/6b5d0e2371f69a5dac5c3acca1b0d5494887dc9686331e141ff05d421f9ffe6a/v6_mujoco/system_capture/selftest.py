"""Fixed synthetic cases and bounded single-state checks; no physical integration."""
import json
import unittest
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from .contracts import *
from .scheduler import ClockSpec
from .registry import controller,ALGORITHMS,NotImplementedStage,scenario,models,thresholds
from .adapters import packet_from_legacy,packet_to_legacy,estimate_from_legacy
from .common import OLD,read

def packet(t=0.,poses=()):
    return SensorPacket(t,t,t,RobotObservation(np.r_[np.zeros(3),1.,np.zeros(3)],np.zeros(3),np.zeros(3),np.zeros(7),np.zeros(7),np.zeros(7)),poses,np.zeros(6),False)

class ContractsTest(unittest.TestCase):
    def test_unimplemented_phase_does_not_run(self):
        import io,contextlib
        from .cli import main
        from .common import OUT
        before=set(OUT.parent.glob('*'))
        with contextlib.redirect_stdout(io.StringIO()) as output:self.assertEqual(main(['--phase','S01','--mode','run']),2)
        self.assertIn('NOT_IMPLEMENTED',output.getvalue());self.assertEqual(before,set(OUT.parent.glob('*')))
    def test_array_snapshot(self):
        x=np.arange(7.);p=replace(packet().robot,q7=x);x[:]=99
        np.testing.assert_array_equal(p.q7,np.arange(7.))
        with self.assertRaises(ValueError):p.q7.setflags(write=True)
    def test_nested_snapshot(self):
        cfg={'nested':{'P':np.eye(2)}};p=ControllerSetup({'mass':20},cfg);cfg['nested']['P'][0,0]=99
        self.assertEqual(p.algorithm_config['nested']['P'][0,0],1)
        with self.assertRaises(TypeError):p.algorithm_config['nested']['x']=0
    def test_covariance_alias_t0(self):
        from v6_mujoco.feasible_capture.estimator_adapter import TimedEstimator
        cfg=read(OLD/'runs/R03_V2_S01/config.json')['mission'];f=TimedEstimator(cfg)
        e=f.update(packet_to_legacy(packet()));s=estimate_from_legacy(e,cfg,'FREE_PROCESS');before=s.P.copy()
        f.P[:]=np.eye(12)*99
        np.testing.assert_array_equal(s.P,before)
        self.assertFalse(s.valid);self.assertEqual(s.t_measurement,-1.)
    def test_rotation_rejects_reflection(self):
        with self.assertRaises(ValueError):rotation(np.diag([1,1,-1]))
    def test_covariance_rejects_negative(self):
        with self.assertRaises(ValueError):covariance(-np.eye(6),6)
    def test_nonfinite_rejected(self):
        with self.assertRaises(ValueError):replace(packet(),t_control=float('nan'))
    def test_future_pose_rejected(self):
        with self.assertRaises(ValueError):PoseObservation(1,0,np.zeros(3),np.eye(3),np.eye(6),FrameConvention('W','target_geometry'))
    def test_packet_clock_order(self):
        with self.assertRaises(ValueError):replace(packet(),t_arrival=1.)
    def test_clock_grid(self):
        c=ClockSpec(.002,.002,.02);self.assertTrue(c.task_tick(.04));self.assertFalse(c.task_tick(.042))
        with self.assertRaises(ValueError):c.task_tick(.041)
        with self.assertRaises(ValueError):ClockSpec(.003,.002,.02)
    def test_units_and_order(self):
        with self.assertRaises(ValueError):FrameConvention('W','flange',units=('mm',))
        with self.assertRaises(ValueError):FrameConvention('W','flange',velocity_order='angular_then_linear')
    def test_point_shift_power(self):
        W=np.array([2.,3.,5.,7.,11.,13.]);v=np.array([.1,.2,.3]);w=np.array([.4,.5,.6]);r=np.array([.7,.8,.9])
        np.testing.assert_allclose(W@np.r_[v,w],shift_wrench(W,r)@np.r_[shift_point_velocity(v,w,r),w],atol=1e-12)
    def test_spatial_twist_conversion(self):
        V=np.array([1,2,3,.1,.2,.3]);p=np.array([2,3,4]);out=spatial_to_point_velocity(V,p)
        np.testing.assert_allclose(out[:3],V[:3]+np.cross(V[3:],p))
    def test_base_body_angular_is_explicit(self):
        q=Rotation.from_euler('z',90,degrees=True).as_quat();r=replace(packet().robot,base_pose_wxyz=np.r_[np.zeros(3),q[3],q[:3]],base_angular_velocity_B=np.array([1.,0,0]))
        np.testing.assert_allclose(r.base_twist_world()[3:],[0,1,0],atol=1e-14)
    def test_packet_roundtrip(self):
        p=packet(.02,(PoseObservation(.01,.02,np.ones(3),np.eye(3),np.eye(6),FrameConvention('W','target_geometry')),))
        self.assertEqual(plain(p),plain(packet_from_legacy(packet_to_legacy(p))))
    def test_json_roundtrip(self):
        p=packet();self.assertEqual(plain(p),plain(decode(json.loads(json.dumps(encode(p))))))
        with self.assertRaises(ValueError):decode({'$type':'eval','fields':{}})
    def test_reference_frame_and_progress(self):
        r=MotionReference(0,np.eye(4),*[np.zeros(3)]*4,0,1,0,0,0,.02)
        self.assertEqual(plain(r),plain(decode(encode(r))))
        with self.assertRaises(ValueError):replace(r,s=8.)
        with self.assertRaises(ValueError):replace(r,valid_until=-1)
    def test_parameter_rank(self):
        p=ParameterEstimate(np.r_[20.,np.zeros(9)],np.eye(10),'local scaled covariance',0,False,{},False)
        with self.assertRaises(ValueError):replace(p,rank=11)
        self.assertEqual(plain(p),plain(decode(encode(p))))
    def test_truth_boundary(self):
        s=scenario('R03_V2_S01');setup=s.controller_setup()
        changed=replace(s,scenario_id='hidden',truth_evaluation_only={'mass':999.}).controller_setup()
        self.assertEqual(plain(setup),plain(changed))
        with self.assertRaises(ValueError):ControllerSetup({}, {'nested':{'truth_evaluation_only':{}}})
        with self.assertRaises(TypeError):controller('n208_baseline',s)
    def test_domains_and_reserved(self):
        self.assertEqual(models()['paper_compat_srs'].status,'NOT_IMPLEMENTED')
        with self.assertRaises(NotImplementedStage):controller('paper_compat_reserved',ControllerSetup({},{}),'paper_compat_srs')
        with self.assertRaises(ValueError):controller('n208_baseline',ControllerSetup({},{}),'paper_compat_srs')
    def test_four_gate_classes(self):
        cs=thresholds(scenario('R01_V2_nominal').algorithm['legacy_mission'])
        self.assertEqual({c.role for c in cs},{'task_safety','sensor_gate','algorithm_acceptance','numerical'})
        self.assertEqual(next(c for c in cs if c.name=='predicted_tracking').role,'algorithm_acceptance')
    def test_abort_not_safe_action(self):
        with self.assertRaises(ValueError):ControlProposal(0,np.zeros(7),'ABORT',{},(),True,'SIM_ABORT','NO_VERIFIED_CONTROL',False)
    def test_n208_single_state_adapter(self):
        from v6_mujoco.adaptive_capture.controller import Controller
        s=scenario('R01_V2_nominal');setup=s.controller_setup();c=Controller(plain(setup.prior),plain(setup.algorithm_config));a=controller('n208_baseline',setup)
        p=packet();tau,latch=c.update(packet_to_legacy(p),False);proposal=a.update(p,False)
        np.testing.assert_array_equal(tau,proposal.tau7);self.assertEqual(latch,proposal.latch_request)
        with self.assertRaises(ValueError):a.update(p,False)

def suite():return unittest.defaultTestLoader.loadTestsFromTestCase(ContractsTest)
