"""Value-only public interfaces. No MjModel, MjData or truth handles cross them."""
from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class PoseObservation:
    stamp: float
    position: np.ndarray
    rotation: np.ndarray
    covariance: np.ndarray

@dataclass(frozen=True)
class SensorPacket:
    time: float
    base_pose: np.ndarray
    base_velocity: np.ndarray
    joint_position: np.ndarray
    joint_velocity: np.ndarray
    actuator_torque: np.ndarray
    wrench_target_at_grasp_world: np.ndarray
    contact: bool
    poses: tuple

@dataclass(frozen=True)
class StateEstimate:
    time: float
    measurement_time: float
    p: np.ndarray
    R: np.ndarray
    v: np.ndarray
    w: np.ndarray
    covariance: np.ndarray
    valid: bool
    innovation: np.ndarray

def skew(x):
    a,b,c=x
    return np.array([[0.,-c,b],[c,0.,-a],[-b,a,0.]])
