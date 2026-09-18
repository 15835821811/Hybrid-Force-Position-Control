"""Continuous geometric arm-shape coordinate for the offset Flexiv SRS-like arm."""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from ..model import FlexivModelSpec, body_id


def _wrap_to_pi(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def unwrap_angle(previous_unwrapped_rad: float, current_wrapped_rad: float) -> float:
    """Continue a wrapped angle without introducing 2*pi jumps."""

    previous = float(previous_unwrapped_rad)
    current = float(current_wrapped_rad)
    return previous + _wrap_to_pi(current - _wrap_to_pi(previous))


@dataclass(frozen=True)
class ArmShapeSample:
    angle_rad: float
    shoulder_position_base_m: np.ndarray
    elbow_position_base_m: np.ndarray
    wrist_position_base_m: np.ndarray
    shoulder_wrist_length_m: float
    elbow_plane_projection_m: float
    reference_projection_norm: float
    singular: bool


class ArmShapeKinematics:
    """S-E-W arm angle defined in the spacecraft-base coordinate frame.

    Flexiv link offsets violate the ideal intersecting-axis assumptions used by
    the paper's analytic arm-angle Jacobian.  A geometric scalar is therefore
    measured from joint2/joint4/joint6 anchors and differentiated numerically.
    The reference base axis is selected once at construction and never switches.
    """

    def __init__(
        self,
        spec: FlexivModelSpec,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        *,
        shoulder_joint: str = "joint2",
        elbow_joint: str = "joint4",
        wrist_joint: str = "joint6",
        singularity_margin: float = 1e-6,
    ) -> None:
        if singularity_margin <= 0.0:
            raise ValueError("singularity margin must be positive")
        self.spec = spec
        self.model = model
        self.singularity_margin = float(singularity_margin)
        self.qpos_ids, _ = spec.joint_addresses(model)
        self.base_body = body_id(model, "base_link_0")
        self.joint_ids = tuple(
            self._joint_id(name) for name in (shoulder_joint, elbow_joint, wrist_joint)
        )
        mujoco.mj_forward(model, data)
        shoulder, _elbow, wrist = self._points_in_base(data)
        direction = wrist - shoulder
        direction /= max(float(np.linalg.norm(direction)), 1e-15)
        axes = np.eye(3)
        self.reference_axis_index = int(np.argmin(np.abs(axes @ direction)))
        self.reference_axis_base = axes[self.reference_axis_index].copy()
        initial = self.sample(data)
        if initial.singular:
            raise ValueError("initial Flexiv arm shape is singular")

    def _joint_id(self, name: str) -> int:
        joint = int(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name))
        if joint < 0:
            raise ValueError(f"joint {name!r} is missing")
        return joint

    def _points_in_base(self, data: mujoco.MjData) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        base_position = np.asarray(data.xpos[self.base_body])
        base_rotation = np.asarray(data.xmat[self.base_body]).reshape(3, 3)
        points = []
        for joint in self.joint_ids:
            world = np.asarray(data.xanchor[joint])
            points.append(base_rotation.T @ (world - base_position))
        return points[0], points[1], points[2]

    def sample(self, data: mujoco.MjData) -> ArmShapeSample:
        shoulder, elbow, wrist = self._points_in_base(data)
        shoulder_wrist = wrist - shoulder
        length = float(np.linalg.norm(shoulder_wrist))
        if length <= self.singularity_margin:
            return ArmShapeSample(0.0, shoulder, elbow, wrist, length, 0.0, 0.0, True)
        axis = shoulder_wrist / length
        elbow_vector = elbow - shoulder
        elbow_projection = elbow_vector - float(elbow_vector @ axis) * axis
        reference_projection = self.reference_axis_base - float(self.reference_axis_base @ axis) * axis
        elbow_norm = float(np.linalg.norm(elbow_projection))
        reference_norm = float(np.linalg.norm(reference_projection))
        singular = (
            elbow_norm <= self.singularity_margin
            or reference_norm <= self.singularity_margin
        )
        if singular:
            angle = 0.0
        else:
            current = elbow_projection / elbow_norm
            reference = reference_projection / reference_norm
            angle = float(
                np.arctan2(
                    axis @ np.cross(reference, current),
                    np.clip(reference @ current, -1.0, 1.0),
                )
            )
        return ArmShapeSample(
            angle_rad=angle,
            shoulder_position_base_m=shoulder,
            elbow_position_base_m=elbow,
            wrist_position_base_m=wrist,
            shoulder_wrist_length_m=length,
            elbow_plane_projection_m=elbow_norm,
            reference_projection_norm=reference_norm,
            singular=singular,
        )

    def jacobian(self, data: mujoco.MjData, step_rad: float = 1e-6) -> np.ndarray:
        step = float(step_rad)
        if not np.isfinite(step) or step <= 0.0:
            raise ValueError("finite-difference step must be finite and positive")
        original_qpos = np.asarray(data.qpos).copy()
        values = np.zeros(len(self.qpos_ids), dtype=np.float64)
        try:
            for column, qpos_id in enumerate(self.qpos_ids):
                data.qpos[:] = original_qpos
                data.qpos[qpos_id] += step
                mujoco.mj_forward(self.model, data)
                plus = self.sample(data)
                data.qpos[:] = original_qpos
                data.qpos[qpos_id] -= step
                mujoco.mj_forward(self.model, data)
                minus = self.sample(data)
                if plus.singular or minus.singular:
                    raise ValueError("arm-angle Jacobian probe reached a shape singularity")
                values[column] = _wrap_to_pi(plus.angle_rad - minus.angle_rad) / (2.0 * step)
        finally:
            data.qpos[:] = original_qpos
            mujoco.mj_forward(self.model, data)
        return values
