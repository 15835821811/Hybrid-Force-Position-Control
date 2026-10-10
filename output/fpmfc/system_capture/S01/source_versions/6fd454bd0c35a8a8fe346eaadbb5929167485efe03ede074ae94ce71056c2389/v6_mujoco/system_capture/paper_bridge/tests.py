"""Independent mathematical checks at the initial and ten preregistered states."""
import time
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from .common import OUT,assumptions,source,save,wrap,sha
from .model import generate,compile_model,initial,MODEL
from .kinematics import IndependentSRS
from .control import EngineShape,SRSHQP,source_velocity
from .reference import Reference,blend,integrate_configuration

def run():
    started=time.perf_counter();cpu=time.process_time();m=compile_model();d=initial(m);k=IndependentSRS();shape_engine=EngineShape(m);q0=d.qpos[7:].copy();tol=assumptions()['numerical_tests'];rows=[]
    for i in range(11):
        d.qpos[7:]=q0 if i==0 else q0+.18*np.sin((i+1)*np.arange(1,8))
        rb=Rotation.from_rotvec(np.array([.02,-.03,.015])*i);d.qpos[:3]=np.array([.013,-.007,.021])*i;d.qpos[3:7]=rb.as_quat()[[3,0,1,2]];mujoco.mj_forward(m,d)
        t=k.terms(d.qpos);s=k.shape(d.qpos[7:]);e=shape_engine.full(d);M=np.zeros((13,13));mujoco.mj_fullM(m,M,d.qM)
        Jp=np.zeros((3,13));Jw=np.zeros((3,13));mujoco.mj_jacSite(m,d,Jp,Jw,0);J=np.vstack((Jp,Jw))
        fd_errors=[];shape_fd=[]
        for h in [tol['finite_difference_step']/2,tol['finite_difference_step'],tol['finite_difference_step']*2]:
            Jfd=np.zeros((6,13));sj=np.zeros(7)
            for j in range(13):
                delta=np.eye(13)[j];plus=d.qpos.copy();minus=d.qpos.copy();mujoco.mj_integratePos(m,plus,delta,h);mujoco.mj_integratePos(m,minus,delta,-h)
                dp=mujoco.MjData(m);dm=mujoco.MjData(m);dp.qpos[:]=plus;dm.qpos[:]=minus;mujoco.mj_forward(m,dp);mujoco.mj_forward(m,dm)
                Jfd[:3,j]=(dp.site_xpos[0]-dm.site_xpos[0])/(2*h)
                Jfd[3:,j]=Rotation.from_matrix(dp.site_xmat[0].reshape(3,3)@dm.site_xmat[0].reshape(3,3).T).as_rotvec()/(2*h)
                if j>=6:sj[j-6]=float(wrap(shape_engine.full(dp).angle-shape_engine.full(dm).angle))/(2*h)
            fd_errors.append(float(abs(J-Jfd).max()));shape_fd.append(float(abs(s.J-sj).max()))
        Pengine=np.vstack((-np.linalg.solve(M[:6,:6],M[:6,6:]),np.eye(7)))
        velocity=np.sin(np.arange(13)+i)*.03
        energy=sum(.5*k.mass[j]*np.linalg.norm(b[2]@velocity)**2+.5*(b[3]@velocity)@b[4]@(b[3]@velocity) for j,b in enumerate(t.bodies))
        momentum=np.zeros(6)
        for j,b in enumerate(t.bodies):
            lin=k.mass[j]*(b[2]@velocity);momentum[:3]+=lin;momentum[3:]+=b[4]@(b[3]@velocity)+np.cross(b[0],lin)
        ref=Reference(t,s.angle,15.6,s.angle).sample(0)
        from dataclasses import replace
        ref=replace(ref,v=np.array([.003,-.002,.001]),w=np.array([.002,.001,-.003]),psidot=.006)
        A=source_velocity(t,s,ref);literal=source_velocity(t,s,ref,literal=True)
        hqp=SRSHQP(m);B=hqp.command(d,ref)
        row={'index':i,'joint_rad':d.qpos[7:].tolist(),'base_pose_wxyz':d.qpos[:7].tolist(),
             'fk_position_m':float(abs(t.p-d.site_xpos[0]).max()),'fk_rotation_rad':float(Rotation.from_matrix(t.R@d.site_xmat[0].reshape(3,3).T).magnitude()),
             'full_mass_matrix_max_abs':float(abs(t.M-M).max()),'full_jacobian_max_abs':float(abs(t.J-J).max()),'finite_difference_jacobian_errors':fd_errors,
             'momentum_zero_map':float(abs(t.momentum_matrix@Pengine).max()),'independent_momentum_sum_error':float(abs(momentum-t.momentum_matrix@velocity).max()),
             'generalized_projection_error':float(abs(t.Jg-J@Pengine).max()),'reaction_map_difference':float(abs(t.P-Pengine).max()),'kinetic_energy_error_j':float(abs(energy-.5*velocity@M@velocity)),
             'shape_engine_angle_error_rad':float(abs(wrap(s.angle-e.angle))),'shape_engine_jacobian_error':float(abs(s.J-e.J).max()),'shape_fd_errors':shape_fd,
             'eq23_l_equals_w_cross_V_error':float(abs(s.J-s.eq23_normal_reading).max()),'eq23_l_equals_projected_LV_error':float(abs(s.J-s.printed_eq23).max()),'Jg_N_norm':A.primary_null_residual,'A_N_world_twist':np.r_[t.A[:3]@A.null,t.base_rotation@t.A[3:]@A.null].tolist(),
             'AB_primary_velocity_error':float(np.linalg.norm(t.Jg@(A.velocity-B.joint_velocity))),'AB_shape_velocity_error':float(abs(s.J@(A.velocity-B.joint_velocity))),
             'hqp_success':bool(B.success),'hqp_hierarchy_linear_error':B.hierarchy_linear_degradation_m_s,'hqp_hierarchy_angular_error':B.hierarchy_angular_degradation_rad_s,
             'literal_eq31_shape_residual_rad_s':literal.shape_residual,'residual_interpretation_shape_residual_rad_s':A.shape_residual,'min_generalized_singular_value':float(A.rank_values[-1])}
        row['passed']=bool(max(row['fk_position_m'],row['fk_rotation_rad'])<tol['fk_m_rad'] and row['full_mass_matrix_max_abs']<1e-10 and row['full_jacobian_max_abs']<tol['jacobian']
          and max(fd_errors)<tol['jacobian'] and row['momentum_zero_map']<tol['momentum_map'] and row['independent_momentum_sum_error']<tol['momentum_map']
          and row['generalized_projection_error']<tol['jacobian'] and row['reaction_map_difference']<tol['momentum_map'] and row['kinetic_energy_error_j']<tol['kinetic_energy_j']
          and row['shape_engine_angle_error_rad']<tol['fk_m_rad'] and row['shape_engine_jacobian_error']<tol['shape_derivative'] and max(shape_fd)<tol['shape_derivative']
          and row['eq23_l_equals_projected_LV_error']<tol['shape_derivative'] and row['Jg_N_norm']<tol['jacobian'] and B.success and row['AB_primary_velocity_error']<tol['hierarchy'] and row['AB_shape_velocity_error']<tol['hierarchy'])
        rows.append(row)
    d=initial(m);t=k.terms(d.qpos);s=k.shape(d.qpos[7:]);reference=Reference(t,s.angle,15.6,.2686)
    reference_checks=[]
    for at in [0.,3.,7.8,12.,15.6]:
        ref=reference.sample(at)
        if at in [0.,15.6]:reference_checks.append({'time':at,'endpoint_derivatives_zero':max(np.linalg.norm(ref.v),np.linalg.norm(ref.w),np.linalg.norm(ref.a),np.linalg.norm(ref.alpha),abs(ref.psidot))<1e-12})
        else:
            h=1e-5;rp=reference.sample(at+h);rm=reference.sample(at-h)
            reference_checks.append({'time':at,'position_derivative_error':float(np.linalg.norm((rp.T[:3,3]-rm.T[:3,3])/(2*h)-ref.v)),
              'rotation_derivative_error':float(np.linalg.norm(Rotation.from_matrix(rp.T[:3,:3]@rm.T[:3,:3].T).as_rotvec()/(2*h)-ref.w))})
    mass=source()['inertia']['mass_kg'];I=np.array(source()['inertia']['diagonal_kg_m2']);radii=np.sqrt(5/(2*np.array(mass)[:,None])*(I.sum(axis=1)[:,None]-2*I))
    periodic_error=float(abs(wrap(np.deg2rad(-179)-np.deg2rad(179))-np.deg2rad(2)))
    nonzero=np.max([np.linalg.norm(x['A_N_world_twist']) for x in rows])
    model_checks={'passed':bool(m.nq==14 and m.nv==13 and m.nu==7 and abs(sum(m.body_mass)-sum(mass))<1e-12 and np.all(radii>0) and np.all(m.opt.gravity==0) and m.neq==0 and not np.any(m.geom_contype)),
      'total_mass_kg':float(sum(m.body_mass)),'joint_inputs':m.nu,'free_base':True,'gravity':m.opt.gravity.tolist(),'inertia_positive_definite':bool(np.all(I>0)),
      'inertia_triangle_conditions':bool(np.all(I.sum(axis=1)[:,None]-2*I>=0)),'equivalent_uniform_ellipsoid_semiaxes_m':radii.tolist(),
      'geometry_inertia_scope':'Table inertias admit positive mass distributions; centerline rendering is schematic, not a mass geometry or collision envelope. Physical clearance SOURCE_LIMITED.',
      'no_target_physical_body':True,'contact_disabled':True,'model_sha256':sha(MODEL)}
    passed=model_checks['passed'] and all(x['passed'] for x in rows) and periodic_error<1e-12 and nonzero>1e-5 and all(x.get('endpoint_derivatives_zero',True) and x.get('position_derivative_error',0)<1e-8 and x.get('rotation_derivative_error',0)<1e-8 for x in reference_checks)
    result={'passed':bool(passed),'states':rows,'reference_checks':reference_checks,'periodic_wrap_error_rad':periodic_error,'non_reactionless_counterexample_max_norm':float(nonzero),
      'eq23_interpretation':'l=(w cross V) cross unit(w)=length(w)*project_perp_w(V) agrees with Eq18 derivative. l=w cross V does not. Source leaves l undefined; both residuals retained. Controller uses independent derivative of Eq18.',
      'eq31_interpretation':'Literal sum double counts primary shape motion in general; total shape tracking uses residual rate under an explicit assumption',
      'tolerances':tol,'cpu_s':time.process_time()-cpu,'wall_s':time.perf_counter()-started,'new_dynamics_attempts':0,'kinematic_rollouts':0}
    save(OUT/'model_checks.json',model_checks);save(OUT/'equation_tests.json',result)
    print({'math_passed':passed,'states':len(rows),'failed_states':[x['index'] for x in rows if not x['passed']],'eq23_projected_l_max_error':max(x['eq23_l_equals_projected_LV_error'] for x in rows),'base_nullspace_motion_max':nonzero},flush=True)
    return bool(passed)
