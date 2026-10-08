"""Geometry-derived boundary jets and three frozen reference candidates."""
from dataclasses import replace
import numpy as np
from scipy.spatial.transform import Rotation
from v6_mujoco.model import default_model_spec
from v6_mujoco.end_to_end_capture.common import preconfig,C1,read
from v6_mujoco.fpmfc.target import target_from_config
from v6_mujoco.fpmfc.handoff_trajectory import ContactHandoffTrajectory
from v6_mujoco.fpmfc.shape import ArmShapeKinematics
from .audit import siteT

def flange_jet(m,d,t):
    E=np.linalg.inv(siteT(m,d,'flange_site'))@siteT(m,d,'postgrasp_tool_interface')
    s=target_from_config(preconfig()).sample(t)
    R=s.grasp_rotation_world@E[:3,:3].T; r=-R@E[:3,3]
    w=s.angular_velocity_world_rad_s; alpha=np.zeros(3)
    p=s.grasp_position_world_m+r;v=s.grasp_linear_velocity_world_m_s+np.cross(w,r)
    a=np.cross(w,np.cross(w,p-s.center_position_world_m))+np.cross(alpha,p-s.center_position_world_m)
    return p,R,v,w,a,alpha

def shape_model(m,d):
    c=preconfig()['shape'];return ArmShapeKinematics(default_model_spec(),m,d,shoulder_joint=c['shoulder_joint'],elbow_joint=c['elbow_joint'],wrist_joint=c['wrist_joint'],singularity_margin=c['singularity_margin'])

class Reference:
    def __init__(self,m,d,name='standard_C1'):
        p,R,v,w,a,alpha=flange_jet(m,d,8.);F=siteT(m,d,'flange_site');self.name=name
        self.base=ContactHandoffTrajectory(F[:3,3],p,F[:3,:3],R,shape_model(m,d).sample(d).angle_rad,read(C1/'pairing_manifest.json')['source_candidate']['terminal_arm_angle_rad'],8.,v,a,w,alpha)
        self.target=target_from_config(preconfig())
    def sample(self,t):
        r=self.base.sample(t)
        if self.name=='standard_C1' or t<=6:return r
        # C2 outward normal bump; zero p/v/a at start and T. It does not
        # move the terminal interface or delay the fixed 8 s capture time.
        amplitude={'normal_approach_40mm':.04,'normal_approach_80mm':.08}[self.name]
        u=(t-6)/2; c=np.array([0,0,0,64,-192,192,-64])*amplitude
        b=np.polynomial.polynomial.polyval(u,c);bd=np.polynomial.polynomial.polyval(u,np.polynomial.polynomial.polyder(c))/2;bdd=np.polynomial.polynomial.polyval(u,np.polynomial.polynomial.polyder(c,2))/4
        target=self.target.sample(t);n=-target.grasp_rotation_world[:,2];w=target.angular_velocity_world_rad_s;nd=np.cross(w,n);ndd=np.cross(w,nd)
        return replace(r,position_world_m=r.position_world_m+b*n,linear_velocity_world_m_s=r.linear_velocity_world_m_s+bd*n+b*nd,linear_acceleration_world_m_s2=r.linear_acceleration_world_m_s2+bdd*n+2*bd*nd+b*ndd)
