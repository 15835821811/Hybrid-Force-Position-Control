"""Bounded synthetic properties; never count these as robot captures."""
import copy
import json
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.integrate import solve_ivp
from v6_mujoco.adaptive_capture.contracts import SensorPacket,PoseObservation,skew
from v6_mujoco.feasible_capture.estimator_adapter import TimedEstimator
from .filter import AccelerationEstimator,propagate,predict_snapshot,white_jerk,left_jacobian
from .budget import interface_jacobian,relative_covariance
from .persistence import Journal,records,completed_inputs,validate
from .common import OLD,read

def config():
    c=read(OLD/'runs/R03_V2_S01/config.json')['mission']
    c['s02_estimator']=dict(linear_jerk_density=.002,angular_jerk_density=.02,contact_multiplier=10.,
        initial_velocity_variance=.1,initial_omega_variance=.1,initial_accel_variance=.01,
        initial_alpha_variance=1.,minimum_samples=3,max_step_s=.002)
    return c

def packet(t,stamp=None,p=None,R=None,contact=False):
    z=() if stamp is None else (PoseObservation(stamp,np.zeros(3) if p is None else p,np.eye(3) if R is None else R,np.diag([1e-10]*3+[1.225e-9]*3)),)
    return SensorPacket(t,np.r_[np.zeros(3),1.,0.,0.,0.],np.zeros(6),np.zeros(7),np.zeros(7),np.zeros(7),np.zeros(6),contact,z)

class FilterProperties(unittest.TestCase):
    def test_legacy_identity_and_predictor_only_diff(self):
        from .common import PROJECT,BASE
        import subprocess
        for path in ('v6_mujoco/feasible_capture/estimator_adapter.py','v6_mujoco/adaptive_capture/state_estimator.py',
                     'v6_mujoco/adaptive_capture/controller.py','v6_mujoco/feasible_capture/reference_shaper.py',
                     'v6_mujoco/feasible_capture/progress_governor.py','v6_mujoco/adaptive_capture/sensors.py',
                     'v6_mujoco/adaptive_capture/plant.py','v6_mujoco/adaptive_capture/evaluation.py'):
            self.assertEqual((PROJECT/path).read_bytes(),subprocess.check_output(['git','show',BASE+':'+path],cwd=PROJECT))
        # Only estimator uncertainty forecast differs in the versioned governor.
        old=(PROJECT/'v6_mujoco/feasible_capture/progress_governor.py').read_text(encoding='utf-8')
        new=(PROJECT/'v6_mujoco/system_capture/estimation/governor.py').read_text(encoding='utf-8')
        expected=old.replace('from .controller_adapter import DiagnosticHQP','from v6_mujoco.feasible_capture.controller_adapter import DiagnosticHQP')
        expected=expected.replace('class ProgressGovernor:', 'class SnapshotProgressGovernor:')
        expected=expected.replace('        for k in range(count+1):','        forecast=e  # Full P18; mean/covariance forecast stays independent of prior plant mean.\n        for k in range(count+1):')
        expected=expected.replace('                _,_,_,_,P=c.state_filter.propagate(e.p,e.R,e.v,e.w,e.covariance,k*dt)','                forecast=c.state_filter.predict_snapshot(forecast,packet.time+k*dt)\n                P=forecast.covariance')
        expected=expected.replace('pe=replace(e,time=packet.time+k*dt,','pe=replace(forecast,time=packet.time+k*dt,')
        self.assertEqual(new,expected)

    def test_guard_uses_raw_estimate_and_public_only_boundary(self):
        from .controller import AccelerationController
        from .common import OLD,PROJECT
        from v6_mujoco.adaptive_capture.known_model import set_measured
        cfg=read(OLD/'runs/R03_V2_S01/config.json');c=AccelerationController(cfg['prior'],config())
        f=c.state_filter;e=f.update(packet(0.,0.));p=packet(.04)
        set_measured(c.model,c.data,p,e)
        c.gate(p,e);first=copy.deepcopy(c.last_gate)
        c.reference.estimate=replace(e,p=e.p+10,valid=True)
        c.gate(p,e);self.assertEqual(first,c.last_gate)
        import ast
        tree=ast.parse((PROJECT/'v6_mujoco/system_capture/estimation/filter.py').read_text(encoding='utf-8'))
        attrs={n.attr for n in ast.walk(tree) if isinstance(n,ast.Attribute)}
        self.assertFalse(attrs.intersection({'qpos','qvel','qacc','body_mass','body_inertia','body_ipos'}))
        self.assertEqual(set(c.state_filter.cfg),set(config()))

    def test_translation_units_and_jerk_integral(self):
        f=AccelerationEstimator(config());s=replace(f.s,v=np.ones(3),a=np.ones(3)*2)
        o=propagate(s,.012)
        np.testing.assert_allclose(o.p,np.ones(3)*(.012+.012**2),atol=1e-15)
        np.testing.assert_allclose(o.v,np.ones(3)*1.024,atol=1e-15)
        # Integrate polynomial impulse response independently by Gauss quadrature.
        x,w=np.polynomial.legendre.leggauss(8);t=(x+1)*.006
        Q=sum(wi*np.outer([ti*ti/2,ti,1],[ti*ti/2,ti,1])*.006*.002**2 for ti,wi in zip(t,w))
        np.testing.assert_allclose(white_jerk(.012,.002),Q,rtol=1e-13,atol=1e-22)

    def test_noncommuting_rotation_and_variational_jacobian(self):
        s=replace(AccelerationEstimator(config()).s,w=np.array([1.2,-.8,.4]),alpha=np.array([2.,3.,-.4]))
        t=.037;o=propagate(s,t)
        ref=solve_ivp(lambda u,r:(skew(s.w+u*s.alpha)@r.reshape(3,3)).ravel(),[0,t],s.R.ravel(),rtol=1e-12,atol=1e-14).y[:,-1].reshape(3,3)
        self.assertLess(np.linalg.norm(o.R-ref),2e-11)
        eps=1e-6;J=np.zeros((18,18))
        for k in range(18):
            def pert(sign):
                dx=np.zeros(18);dx[k]=eps*sign
                q=replace(s,p=s.p+dx[:3],R=Rotation.from_rotvec(dx[3:6]).as_matrix()@s.R,v=s.v+dx[6:9],w=s.w+dx[9:12],a=s.a+dx[12:15],alpha=s.alpha+dx[15:])
                r=propagate(q,t)
                return np.r_[r.p-o.p,Rotation.from_matrix(r.R@o.R.T).as_rotvec(),r.v-o.v,r.w-o.w,r.a-o.a,r.alpha-o.alpha]
            J[:,k]=(pert(1)-pert(-1))/(2*eps)
        # Difference of two covariances removes process Q; retains full cross terms.
        A=np.random.default_rng(8).normal(size=(18,18));P=A@A.T*1e-6
        p=replace(s,full_covariance=P,covariance=P[:12,:12]);z=replace(s,full_covariance=np.zeros((18,18)),covariance=np.zeros((12,12)))
        np.testing.assert_allclose(propagate(p,t).full_covariance-propagate(z,t).full_covariance,J@P@J.T,atol=2e-13)

    def test_left_reset_jacobian_and_quaternion_sign(self):
        x=np.array([.1,-.2,.05]);eps=1e-6;J=np.zeros((3,3))
        for k in range(3):
            d=np.eye(3)[k]*eps
            J[:,k]=(Rotation.from_matrix(Rotation.from_rotvec(x+d).as_matrix()@Rotation.from_rotvec(-x).as_matrix()).as_rotvec()-Rotation.from_matrix(Rotation.from_rotvec(x-d).as_matrix()@Rotation.from_rotvec(-x).as_matrix()).as_rotvec())/(2*eps)
        np.testing.assert_allclose(left_jacobian(x),J,atol=1e-10)
        q=Rotation.from_rotvec(x).as_quat();np.testing.assert_allclose(Rotation.from_quat(q).as_matrix(),Rotation.from_quat(-q).as_matrix())

    def test_snapshot_psd_marginal_segmentation_and_no_pollution(self):
        f=AccelerationEstimator(config());s=replace(f.s,w=np.array([.2,-.4,.1]),alpha=np.array([.1,.2,.3]))
        old=s.full_covariance.copy();o=predict_snapshot(s,.04);two=predict_snapshot(predict_snapshot(s,.02),.04)
        np.testing.assert_allclose(o.full_covariance,two.full_covariance,atol=1e-13)
        np.testing.assert_array_equal(s.full_covariance,old)
        np.testing.assert_array_equal(o.covariance,o.full_covariance[:12,:12])
        self.assertGreaterEqual(np.linalg.eigvalsh(o.full_covariance).min(),-1e-13)
        np.testing.assert_allclose(predict_snapshot(s,1e-10).p,s.p+s.v*1e-10,atol=1e-15)
        with self.assertRaises(ValueError):s.full_covariance.flags.writeable=True
        P=old.copy();P[12:15,12:15]+=np.eye(3)
        bigger=predict_snapshot(replace(s,full_covariance=P,covariance=P[:12,:12]),.04)
        np.testing.assert_allclose(bigger.covariance[6:9,6:9]-o.covariance[6:9,6:9],np.eye(3)*.04**2,atol=1e-13)

    def test_causality_missing_duplicate_old_nonfinite(self):
        f=AccelerationEstimator(config())
        with self.assertRaises(ValueError):f.update(packet(.01,.012))
        f.update(packet(.012,0.));f.update(packet(.018,.006));e=f.update(packet(.024,.012))
        self.assertTrue(e.valid);self.assertEqual(e.measurement_time,.012)
        out=f.update(packet(.026,.006));self.assertEqual(f.n,3);self.assertTrue(f.rejects)
        old=out.covariance.copy();out=f.update(packet(.08));self.assertFalse(out.valid)
        self.assertGreater(np.trace(out.covariance),np.trace(old))
        f.update(packet(.5,.1));self.assertEqual(f.rejects[-1]['reason'],'OUTSIDE_MODE_BUFFER')
        with self.assertRaises(ValueError):f.update(packet(.6,.59,p=np.array([np.nan,0.,0.])))

    def test_history_contact_not_rewritten(self):
        f=AccelerationEstimator(config());f.update(packet(0.,0.));f.update(packet(.01,contact=False))
        before=f.s;f.update(packet(.02,contact=True))
        hist=f.between(before,0.,.03)
        expected=propagate(propagate(before,.02,False),.01,True)
        np.testing.assert_allclose(hist.full_covariance,expected.full_covariance,atol=1e-14)

    def test_reference_point_jacobian(self):
        s=replace(AccelerationEstimator(config()).s,w=np.array([.3,-.4,.5]),R=Rotation.from_rotvec([.4,.2,-.3]).as_matrix());r=np.array([.12,-.03,.08])
        J=interface_jacobian(s.R,s.w,r);eps=1e-6
        def values(dx):
            R=Rotation.from_rotvec(dx[3:6]).as_matrix()@s.R
            return np.r_[s.p+dx[:3]+R@r,dx[3:6],s.v+dx[6:9]+np.cross(s.w+dx[9:12],R@r),s.w+dx[9:12]]
        num=np.column_stack([(values(np.eye(12)[i]*eps)-values(-np.eye(12)[i]*eps))/(2*eps) for i in range(12)])
        np.testing.assert_allclose(J,num,atol=1e-10)
        P=relative_covariance(s,r,robot_R=s.R,robot_w=s.w,offset_tool=r,robot_P=s.covariance,cross_P=s.covariance)
        np.testing.assert_allclose(P,0.,atol=1e-14)

class PersistenceProperties(unittest.TestCase):
    results=[]
    def test_faults(self):
        for fault in ('metrics_serialization','plot_exception','integration_exception','block_write','rename'):
            with self.subTest(fault=fault),tempfile.TemporaryDirectory() as td:
                path=Path(td)/'journal';j=Journal(path,block_records=3)
                j.append(dict(kind='input_applied',step=0,time=0.,tau=[0.]*7))
                j.append(dict(kind='step_completed',step=0,time=.002,qpos=[0.],qvel=[0.]))
                j.commit()
                try:
                    if fault=='metrics_serialization':json.dumps({'bad':object()})
                    elif fault=='plot_exception':raise RuntimeError('injected plot failure')
                    elif fault=='integration_exception':
                        j.append(dict(kind='input_applied',step=1,time=.002,tau=[0.]*7));raise RuntimeError('injected integration failure')
                    elif fault=='block_write':
                        j.append(dict(kind='proposal',time=.002))
                        with patch('v6_mujoco.system_capture.estimation.persistence.atomic_bytes',side_effect=OSError('write failure')):j.commit()
                    else:
                        j.append(dict(kind='proposal',time=.002))
                        with patch('v6_mujoco.system_capture.estimation.persistence.os.replace',side_effect=OSError('rename failure')):j.commit()
                except (TypeError,RuntimeError,OSError):pass
                finally:
                    # Ordinary exception closes available tail; persistent I/O
                    # failure is modeled separately, retaining old committed data.
                    if fault not in ('block_write','rename'):j.close()
                result=validate(path);self.assertEqual(result['completed_steps'],1)
                self.assertEqual(result['max_verifiable_time_s'],.002)
                self.results.append(dict(fault=fault,**result,formal_robot_run=False))
                if fault=='rename':self.assertTrue(list(path.glob('*.tmp')))

    def test_uncommitted_tail_and_corruption_not_repaired(self):
        with tempfile.TemporaryDirectory() as td:
            path=Path(td)/'journal';j=Journal(path,10)
            j.append(dict(kind='proposal',time=0.));j.commit()
            j.append(dict(kind='input_applied',step=0,time=0.,tau=[0.]*7))
            self.assertEqual(len(list(records(path))),1) # simulated process loss
            block=path/j.index['blocks'][0]['file'];block.write_bytes(b'corrupted')
            with self.assertRaises(RuntimeError):list(records(path))

def suite():return unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromTestCase(c) for c in (FilterProperties,PersistenceProperties)])
