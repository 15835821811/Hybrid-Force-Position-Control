"""Optional brake-strength predictions on independent prior/estimated models."""
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from v6_mujoco.model import body_id,default_model_spec
from v6_mujoco.postgrasp.physics import interface_observation
from .known_model import compile_known,set_measured
from .momentum_regressor import tensor,parameters

def put_inertia(m,pi):
    b=body_id(m,'tumbling_target');mass=pi[0];c=pi[1:4]/mass
    Ic=tensor(pi)-mass*((c@c)*np.eye(3)-np.outer(c,c));values,R=np.linalg.eigh(Ic)
    if np.linalg.det(R)<0:R[:,0]*=-1
    q=Rotation.from_matrix(R).as_quat();m.body_mass[b]=mass;m.body_ipos[b]=c;m.body_inertia[b]=values;m.body_iquat[b]=np.r_[q[3],q[:3]]
    mujoco.mj_setConst(m,mujoco.MjData(m))

class BrakeGovernor:
    def __init__(self,prior,damping):
        self.models=[compile_known(prior) for _ in range(3)];self.damping=damping;self.alpha=1.;self.log=[];self.spec=default_model_spec();self.center=None
        pi=parameters(prior['mass_kg'],prior['com_m'],prior['inertia_com_kg_m2'])
        put_inertia(self.models[1][0],pi*.5);put_inertia(self.models[2][0],pi*2.)
    def choose(self,packet,e,pi):
        if self.center is None or np.max(abs(pi-self.center))>1e-10:put_inertia(self.models[0][0],pi);self.center=pi.copy()
        candidates=sorted(set([min(1.,self.alpha+.04),self.alpha,max(0.,self.alpha-.04)]),reverse=True)
        results=[];chosen=None
        for alpha in candidates:
            safe=True;peak=0.;maxp=0.;maxr=0.
            for m,d in self.models:
                set_measured(m,d,packet,e);d.eq_active[0]=True;qj,vj=self.spec.joint_addresses(m)
                for step in range(11):
                    d.ctrl[:]=np.clip(-alpha*self.damping*d.qvel[vj],-self.spec.torque_limits_nm,self.spec.torque_limits_nm);mujoco.mj_forward(m,d)
                    obs=interface_observation(m,d,'postgrasp_latch','postgrasp_tool_interface','target_grasp_site')
                    # Fixed site relation is identity. Euclidean gap / physical rotation.
                    it=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,'postgrasp_tool_interface');ig=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,'target_grasp_site')
                    p=np.linalg.norm(d.site_xpos[it]-d.site_xpos[ig]);r=Rotation.from_matrix(d.site_xmat[it].reshape(3,3).T@d.site_xmat[ig].reshape(3,3)).magnitude()
                    peak=max(peak,obs['load_fraction']);maxp=max(maxp,p);maxr=max(maxr,r)
                    safe &= bool(obs['load_fraction']<=.8 and p<=.0004 and r<=np.deg2rad(.08) and np.all(abs(d.qvel[vj])<=self.spec.velocity_limits_rad_s))
                    if not safe:break
                    if step<10:mujoco.mj_step(m,d)
                if not safe:break
            results.append({'alpha':alpha,'safe_in_sampled_models':safe,'peak_rho':peak,'translation_m':maxp,'rotation_rad':maxr})
            if safe:chosen=alpha;break
        self.log.append({'time':packet.time,'model_pi':pi.tolist(),'predictions':results,'chosen':chosen,'scope':'20 ms finite prior ensemble; no all-uncertainty proof'})
        if chosen is not None:self.alpha=chosen
        return chosen is not None
