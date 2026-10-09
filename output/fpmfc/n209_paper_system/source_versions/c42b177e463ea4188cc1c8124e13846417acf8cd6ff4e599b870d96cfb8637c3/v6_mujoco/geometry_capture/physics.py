"""Physical-system accounting excludes the prescribed visualization marker.

No mass, state, actuator or force in the MuJoCo plant is modified. N205's
bookkeeping included a positive-mass mocap marker in COM/I_lock; this observer
defines the free robot + free target system explicitly and tests it by energy.
"""
import numpy as np
import mujoco
from v6_mujoco.postgrasp.physics import body_jacobian,mass_matrix

def momenta(model,data):
    masses=model.body_mass.copy();excluded=np.where(model.body_mocapid>=0)[0];masses[excluded]=0
    total=float(masses.sum());positions=data.xipos;center=np.sum(masses[:,None]*positions,axis=0)/total
    P=np.zeros(3);H=np.zeros(3);Ilock=np.zeros((3,3));velocities={}
    for b in range(1,model.nbody):
        if not masses[b]:continue
        v=body_jacobian(model,data,b,positions[b])@data.qvel;R=data.ximat[b].reshape(3,3);I=R@np.diag(model.body_inertia[b])@R.T;r=positions[b]-center
        P+=masses[b]*v[:3];H+=I@v[3:]+np.cross(r,masses[b]*v[:3]);Ilock+=I+masses[b]*((r@r)*np.eye(3)-np.outer(r,r))
        velocities[mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,b)]={'linear_velocity_world_m_s':v[:3].tolist(),'angular_velocity_world_rad_s':v[3:].tolist()}
    locked=np.linalg.solve(Ilock,H);T=.5*data.qvel@mass_matrix(model,data)@data.qvel
    return {'mass_kg':total,'center_world_m':center.tolist(),'linear_momentum_world_kg_m_s':P.tolist(),'angular_momentum_about_center_world_kg_m2_s':H.tolist(),
            'locked_inertia_world_kg_m2':Ilock.tolist(),'omega_locked_prediction_world_rad_s':locked.tolist(),'kinetic_energy_j':float(T),
            'relative_energy_j':float(T-P@P/(2*total)-.5*H@locked),'body_velocities':velocities,
            'excluded_visual_mocap_mass_kg':float(model.body_mass[excluded].sum()),'system_definition':'free robot and target; prescribed visual mocap marker excluded; plant unchanged'}
