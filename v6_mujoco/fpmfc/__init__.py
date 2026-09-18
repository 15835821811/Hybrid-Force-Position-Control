"""Adaptive FPMFC reproduction for the free-floating Flexiv Rizon 4s."""

from .config import DEFAULT_CONFIG_PATH, load_fpmfc_config, validate_fpmfc_config
from .dynamics import FreeFloatingKinematics
from .shape import ArmShapeKinematics, ArmShapeSample, unwrap_angle
from .target import PrescribedTumblingTarget, TargetSample, target_from_config
from .trajectory import PoseShapeSample, PoseShapeTrajectory, quintic_time_scaling

__all__ = [
    "ArmShapeKinematics",
    "ArmShapeSample",
    "DEFAULT_CONFIG_PATH",
    "FreeFloatingKinematics",
    "PoseShapeSample",
    "PoseShapeTrajectory",
    "PrescribedTumblingTarget",
    "TargetSample",
    "load_fpmfc_config",
    "quintic_time_scaling",
    "target_from_config",
    "unwrap_angle",
    "validate_fpmfc_config",
]
