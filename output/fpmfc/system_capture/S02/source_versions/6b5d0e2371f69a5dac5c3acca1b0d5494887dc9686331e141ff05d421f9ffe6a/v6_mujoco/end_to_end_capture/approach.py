"""Unchanged C1 HQP/servo evaluated entirely on isolated state copies."""
import copy
from dataclasses import asdict
import time
import numpy as np
from v6_mujoco.model import default_model_spec
from v6_mujoco.run import servo_torque
from v6_mujoco.collision import build_collision_pairs
from v6_mujoco.fpmfc.controller import FPMFCHQP
from v6_mujoco.fpmfc.shape import ArmShapeKinematics
from v6_mujoco.fpmfc.run_capture import DynamicRunConfig,_controller_config,_constraint_config
from v6_mujoco.fpmfc.run_handoff_precontact import _trajectory
from .common import preconfig,read,C1

class Approach:
    def __init__(self,m,real):
        self.model=m;self.spec=default_model_spec();cfg=preconfig();self.run=DynamicRunConfig()
        self.trajectory=_trajectory(read(C1/"pairing_manifest.json"),"C1")
        c=cfg["shape"];self.shape=ArmShapeKinematics(self.spec,m,copy.copy(real),shoulder_joint=c["shoulder_joint"],elbow_joint=c["elbow_joint"],wrist_joint=c["wrist_joint"],singularity_margin=c["singularity_margin"])
        self.hqp=FPMFCHQP(self.spec,m,build_collision_pairs(m),self.shape,controller_config=_controller_config(cfg),constraint_config=_constraint_config(cfg))
        self.qids,self.vids=self.spec.joint_addresses(m);_,self.base=self.spec.base_slices(m)
        self.reference_q=real.qpos[self.qids].copy();self.velocity=np.zeros(7);self.start=np.zeros(7);self.segment_step=0
        self.mass=np.zeros((m.nv,m.nv));self.tasks=[];self.timings=[]
        self.reference_dq=np.zeros(7);self.reference_ddq=np.zeros(7)

    def update(self,real,task_tick):
        started=time.perf_counter();scratch=copy.copy(real)
        if task_tick:
            r=self.trajectory.sample(float(real.time));result=self.hqp.solve_fpmfc(scratch,target_position=r.position_world_m,target_velocity=r.linear_velocity_world_m_s,target_rotation=r.rotation_world,target_angular_velocity=r.angular_velocity_world_rad_s,target_arm_angle_rad=r.arm_angle_rad,target_arm_angle_velocity_rad_s=r.arm_angle_velocity_rad_s)
            self.start=self.velocity.copy();self.velocity=result.joint_velocity.copy();self.segment_step=0
            task={k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in asdict(result).items()}
            task.update(time_s=float(real.time),reference_position=r.position_world_m.tolist(),reference_rotation=r.rotation_world.tolist(),reference_arm=r.arm_angle_rad,command_acceleration=self.hqp.previous_acceleration.tolist(),bound_conflicts=self.hqp.last_bound_conflicts.tolist())
            self.tasks.append(task)
        self.reference_dq=self.start+(self.segment_step+1)/10*(self.velocity-self.start)
        self.reference_ddq=(self.velocity-self.start)/.02
        self.reference_q=np.clip(self.reference_q+self.reference_dq*.002,self.hqp.joint_lower,self.hqp.joint_upper)
        self.reference_q=np.clip(self.reference_q,real.qpos[self.qids]-self.run.reference_tracking_band_rad,real.qpos[self.qids]+self.run.reference_tracking_band_rad)
        torque,_=servo_torque(self.model,scratch,self.vids,self.base,self.qids,self.reference_q,self.reference_dq,self.reference_ddq,self.spec.torque_limits_nm,self.run.servo_natural_frequency_rad_s,self.run.servo_acceleration_limit_rad_s2,self.mass)
        self.segment_step+=1;self.timings.append(time.perf_counter()-started)
        return torque
