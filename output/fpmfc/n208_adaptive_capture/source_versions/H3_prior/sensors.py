"""Only frontend touches true solver state; exposes declared sensor values."""
import copy
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from v6_mujoco.model import default_model_spec,body_id,site_id
from v6_mujoco.postgrasp.physics import joint_slices,body_jacobian
from v6_mujoco.end_to_end_capture.adapter import pads
from .contracts import SensorPacket,PoseObservation

class SensorFrontend:
    def __init__(self,private_sensor_config):
        self.cfg=private_sensor_config;self.queue=[];self.next_pose=0;self.last_pose_index=-1
    def noise(self,index,channel,size):
        return np.random.default_rng(np.random.SeedSequence([self.cfg['seed'],index,channel])).normal(size=size)
    def sample(self,m,real):
        d=copy.copy(real);mujoco.mj_forward(m,d);t=float(real.time);c=self.cfg
        index=int(round(t/.002));b=body_id(m,'tumbling_target')
        pose_index=int(np.floor((t+1e-10)/c['pose_period_s']))
        if pose_index>self.last_pose_index:
            self.last_pose_index=pose_index
            p=d.xpos[b]+c['position_sigma_m']*self.noise(pose_index,1,3)+np.array(c['position_bias_m'])
            R=Rotation.from_rotvec(c['rotation_sigma_rad']*self.noise(pose_index,2,3)+np.array(c['rotation_bias_rad'])).as_matrix()@d.xmat[b].reshape(3,3)
            cov=np.diag([max(c['position_sigma_m'],1e-9)**2]*3+[max(c['rotation_sigma_rad'],1e-9)**2]*3)
            self.queue.append(PoseObservation(t,p.copy(),R.copy(),cov))
        delivered=[]
        while self.queue and self.queue[0].stamp+c['delay_s']<=t+1e-10:delivered.append(self.queue.pop(0))
        # Generalized constraint load in target's six free coordinates is an
        # ideal interface F/T equivalent. It contains no acceleration or inertia.
        g=site_id(m,'target_grasp_site');_,tv=joint_slices(m,'target_free_joint')
        J=body_jacobian(m,d,b,d.site_xpos[g])
        wrench=np.linalg.solve(J[:,tv].T,d.qfrc_constraint[tv])
        wrench+=np.r_[c['force_sigma_n']*self.noise(index,3,3),c['moment_sigma_nm']*self.noise(index,4,3)]
        pair=set(pads(m));contact=any({int(d.contact[i].geom1),int(d.contact[i].geom2)}==pair for i in range(d.ncon))
        spec=default_model_spec();bq,bv=spec.base_slices(m);qj,vj=spec.joint_addresses(m)
        return SensorPacket(t,d.qpos[bq].copy(),d.qvel[bv].copy(),d.qpos[qj].copy(),d.qvel[vj].copy(),d.actuator_force.copy(),wrench,contact,tuple(delivered))
