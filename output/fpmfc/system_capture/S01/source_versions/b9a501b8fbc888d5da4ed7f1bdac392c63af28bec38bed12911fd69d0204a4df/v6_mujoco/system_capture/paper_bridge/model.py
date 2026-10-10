"""SRS MJCF assembled directly from Table 2/3 plus declared assumptions."""
from dataclasses import dataclass
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from .common import PROJECT,source,assumptions

MODEL=PROJECT/'models/paper_compat/srs.xml'
def numbers(x):return ' '.join(format(float(v),'.17g') for v in x)
def generate():
    s=source();a=assumptions();dh=s['dh'];inertia=s['inertia']
    root=ET.Element('mujoco',model='paper_compat_srs_declared_assumptions')
    ET.SubElement(root,'compiler',angle='radian',inertiafromgeom='false',fusestatic='false')
    ET.SubElement(root,'option',timestep=str(a['controller']['physics_dt_s']),gravity='0 0 0',integrator='RK4',tolerance='1e-12',iterations='100')
    default=ET.SubElement(root,'default');ET.SubElement(default,'joint',damping='0',armature='0',frictionloss='0')
    ET.SubElement(default,'geom',contype='0',conaffinity='0',mass='0',rgba='0.5 0.65 0.8 1')
    world=ET.SubElement(root,'worldbody');base=ET.SubElement(world,'body',name='srs_base',pos='0 0 0')
    ET.SubElement(base,'freejoint',name='base_free')
    ET.SubElement(base,'inertial',pos='0 0 0',mass='500',diaginertia='50 50 50')
    # Thin centerline graphics are explicitly schematic, never collision/mass geometry.
    ET.SubElement(base,'geom',type='box',size='0.10 0.10 0.08',rgba='.3 .4 .5 .5')
    angle=a['model']['mount_rotation_x_rad']
    parent=ET.SubElement(base,'body',name='DH0',quat=numbers([np.cos(angle/2),np.sin(angle/2),0,0]))
    for i in range(7):
        off=np.deg2rad(dh['theta_table_deg'][i]);alpha=np.deg2rad(dh['alpha_deg'][i]);d=dh['d_m'][i]
        body=ET.SubElement(parent,'body',name=f'B{i+1}',quat=numbers([np.cos(off/2),0,0,np.sin(off/2)]))
        limit=a['controller']['joint_position_limit_rad']
        ET.SubElement(body,'joint',name=f'srs_joint{i+1}',type='hinge',axis='0 0 1',limited='true',range=numbers([-limit,limit]))
        ET.SubElement(body,'inertial',pos=numbers([0,0,d/2]),mass=str(inertia['mass_kg'][i+1]),diaginertia=numbers(inertia['diagonal_kg_m2'][i+1]))
        if d:
            ET.SubElement(body,'geom',type='capsule',fromto=numbers([0,0,0,0,0,d]),size='.008')
        else:ET.SubElement(body,'geom',type='sphere',size='.012')
        parent=ET.SubElement(body,'body',name=f'DH{i+1}',pos=numbers([0,0,d]),quat=numbers([np.cos(alpha/2),np.sin(alpha/2),0,0]))
    ET.SubElement(parent,'site',name='flange_site',size='.008',rgba='1 .3 .1 1')
    actuators=ET.SubElement(root,'actuator')
    for i in range(7):ET.SubElement(actuators,'motor',name=f'tau{i+1}',joint=f'srs_joint{i+1}',gear='1',ctrllimited='true',ctrlrange=numbers([-a['controller']['torque_limit_nm'],a['controller']['torque_limit_nm']]))
    ET.indent(root);MODEL.parent.mkdir(parents=True,exist_ok=True)
    MODEL.write_text(ET.tostring(root,encoding='unicode')+'\n',encoding='utf-8',newline='\n')
    return MODEL

@dataclass
class SRSSpec:
    joint_names=tuple('srs_joint'+str(i) for i in range(1,8))
    def joint_addresses(self,m):
        ids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n) for n in self.joint_names]
        return m.jnt_qposadr[ids],m.jnt_dofadr[ids]
    def base_slices(self,m):return slice(0,7),slice(0,6)
    @property
    def velocity_limits_rad_s(self):return np.full(7,assumptions()['controller']['joint_velocity_limit_rad_s'])
    @property
    def acceleration_limits_rad_s2(self):return np.full(7,assumptions()['controller']['joint_acceleration_limit_rad_s2'])

def compile_model(dt=None):
    m=mujoco.MjModel.from_xml_string(MODEL.read_text(encoding='utf-8'))
    if dt is not None:m.opt.timestep=dt
    return m
def initial(m):
    d=mujoco.MjData(m);d.qpos[:7]=[0,0,0,1,0,0,0]
    d.qpos[7:]=np.deg2rad(source()['initial']['joint_angles_deg']);d.qvel[:]=0
    mujoco.mj_forward(m,d);return d
