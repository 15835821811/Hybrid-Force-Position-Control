"""One deterministic constrained velocity QP per 50 Hz task tick."""

from __future__ import annotations

import time
from dataclasses import dataclass

import mujoco
import numpy as np
from scipy.linalg import cho_factor, cho_solve

from .collision import CollisionPair
from .model import FlexivModelSpec, site_id


def _clip_norm(value: np.ndarray, limit: float) -> np.ndarray:
    norm = float(np.linalg.norm(value))
    return value if norm <= limit else value * (limit / max(norm, 1e-12))


def smooth_saturate_norm(
    value: np.ndarray,
    limit: float,
    transition_ratio: float = 0.90,
) -> np.ndarray:
    """C2 radial saturation with an identity region and an asymptotic bound.

    The magnitude is unchanged up to ``transition_ratio * limit``.  Above that
    shoulder it follows a scaled hyperbolic tangent whose value, slope, and
    curvature match the identity map at the join.  Direction is preserved and
    the returned norm never exceeds ``limit``.
    """

    vector = np.asarray(value, dtype=np.float64)
    if not np.isfinite(limit) or limit <= 0.0:
        raise ValueError("saturation limit must be finite and positive")
    if not np.isfinite(transition_ratio) or not 0.0 < transition_ratio < 1.0:
        raise ValueError("saturation transition ratio must be in (0, 1)")
    norm = float(np.linalg.norm(vector))
    shoulder = transition_ratio * limit
    if norm <= shoulder:
        return vector.copy()
    width = limit - shoulder
    magnitude = shoulder + width * np.tanh((norm - shoulder) / width)
    return vector * (magnitude / max(norm, 1e-12))


def jerk_limited_stopping_distance(
    speed: float,
    acceleration_toward_limit: float,
    acceleration_limit: float,
    jerk_limit: float,
) -> float:
    """Distance needed to stop with maximum jerk followed by maximum braking.

    ``speed`` and ``acceleration_toward_limit`` use a scalar coordinate whose
    positive direction points toward the joint limit.  The braking policy first
    applies ``-jerk_limit`` and, when reached, holds ``-acceleration_limit``.
    """

    velocity = max(float(speed), 0.0)
    acceleration = float(
        np.clip(acceleration_toward_limit, -acceleration_limit, acceleration_limit)
    )
    if velocity <= 0.0:
        return 0.0
    ramp_duration = (acceleration + acceleration_limit) / jerk_limit
    velocity_after_ramp = (
        velocity
        + acceleration * ramp_duration
        - 0.5 * jerk_limit * ramp_duration**2
    )
    if velocity_after_ramp <= 0.0:
        stop_time = (
            acceleration
            + np.sqrt(acceleration**2 + 2.0 * jerk_limit * velocity)
        ) / jerk_limit
        return max(
            0.0,
            velocity * stop_time
            + 0.5 * acceleration * stop_time**2
            - jerk_limit * stop_time**3 / 6.0,
        )
    ramp_distance = (
        velocity * ramp_duration
        + 0.5 * acceleration * ramp_duration**2
        - jerk_limit * ramp_duration**3 / 6.0
    )
    return max(
        0.0,
        ramp_distance + velocity_after_ramp**2 / (2.0 * acceleration_limit),
    )


def jerk_limited_safe_speed(
    distance: float,
    acceleration_toward_limit: float,
    speed_limit: float,
    acceleration_limit: float,
    jerk_limit: float,
) -> float:
    """Largest nonnegative speed whose emergency stop fits in ``distance``."""

    available = max(float(distance), 0.0)
    maximum = max(float(speed_limit), 0.0)
    if jerk_limited_stopping_distance(
        maximum, acceleration_toward_limit, acceleration_limit, jerk_limit
    ) <= available:
        return maximum
    lower, upper = 0.0, maximum
    for _ in range(36):
        midpoint = 0.5 * (lower + upper)
        if jerk_limited_stopping_distance(
            midpoint,
            acceleration_toward_limit,
            acceleration_limit,
            jerk_limit,
        ) <= available:
            lower = midpoint
        else:
            upper = midpoint
    return lower


def rotation_error_vector_world(target: np.ndarray, current: np.ndarray) -> np.ndarray:
    delta = np.asarray(target).reshape(3, 3) @ np.asarray(current).reshape(3, 3).T
    cosine = float(np.clip(0.5 * (np.trace(delta) - 1.0), -1.0, 1.0))
    angle = float(np.arccos(cosine))
    skew = np.asarray([delta[2, 1] - delta[1, 2], delta[0, 2] - delta[2, 0], delta[1, 0] - delta[0, 1]])
    if angle < 1e-8:
        return 0.5 * skew
    if np.pi - angle < 1e-5:
        values, vectors = np.linalg.eig(delta)
        axis = np.real(vectors[:, int(np.argmin(np.abs(values - 1.0)))])
        return angle * axis / max(float(np.linalg.norm(axis)), 1e-12)
    return 0.5 * angle * skew / np.sin(angle)


@dataclass(frozen=True)
class QPConfig:
    task_period_s: float = 0.020
    position_gain: float = 5.0
    orientation_gain: float = 8.0
    position_weight: float = 500.0
    orientation_weight: float = 4000.0
    posture_weight: float = 0.02
    base_reaction_weight: float = 0.05
    posture_gain: float = 0.08
    linear_speed_limit_m_s: float = 0.24
    angular_speed_limit_rad_s: float = 0.45
    angular_saturation_transition_ratio: float = 0.90
    velocity_limit_scale: float = 0.70
    joint_jerk_limit_rad_s3: float = 80.0
    joint_position_margin_rad: float = 0.04
    joint_barrier_gain: float = 4.0
    clearance_safe_m: float = 0.040
    clearance_activation_m: float = 0.120
    clearance_barrier_gain: float = 12.0
    clearance_query_max_m: float = 0.140
    qp_max_iterations: int = 1200
    qp_tolerance: float = 5e-5
    feasibility_tolerance: float = 1e-4
    admm_rho: float = 50.0
    admm_sigma: float = 1e-6
    admm_relaxation: float = 1.6

    def validate(self) -> None:
        if abs(self.task_period_s - 0.020) > 1e-12:
            raise ValueError("task period must remain 20 ms")
        if not 0.0 < self.clearance_safe_m < self.clearance_activation_m < self.clearance_query_max_m:
            raise ValueError("invalid signed-distance barrier thresholds")
        if not 0.0 < self.admm_relaxation < 2.0:
            raise ValueError("ADMM relaxation must be in (0, 2)")
        if not 0.0 < self.angular_saturation_transition_ratio < 1.0:
            raise ValueError("angular saturation transition ratio must be in (0, 1)")
        if (
            not np.isfinite(self.joint_jerk_limit_rad_s3)
            or self.joint_jerk_limit_rad_s3 <= 0.0
        ):
            raise ValueError("joint jerk limit must be finite and positive")


@dataclass(frozen=True)
class QPResult:
    joint_velocity: np.ndarray
    success: bool
    status: str
    iterations: int
    full_latency_s: float
    solver_latency_s: float
    position_error_m: float
    orientation_error_rad: float
    velocity_residual_m_s: float
    angular_velocity_residual_rad_s: float
    minimum_queried_clearance_m: float
    active_clearance_constraints: int
    binding_clearance_constraints: int
    minimum_constraint_slack: float
    reaction_momentum_residual_norm: float


class HierarchicalVelocityQP:
    """V6 weighted hierarchy adapted from 17 planner coordinates to seven joints."""

    def __init__(self, spec: FlexivModelSpec, model: mujoco.MjModel, pairs: tuple[CollisionPair, ...], config: QPConfig = QPConfig()) -> None:
        config.validate()
        self.spec = spec
        self.model = model
        self.pairs = pairs
        self.config = config
        self.qpos_ids, self.dof_ids = spec.joint_addresses(model)
        _, self.base_dof_slice = spec.base_slices(model)
        self.flange_site = site_id(model, "flange_site")
        self.previous_velocity = np.zeros(7)
        self.previous_acceleration = np.zeros(7)
        self.last_velocity_lower = np.zeros(7)
        self.last_velocity_upper = np.zeros(7)
        self.last_raw_velocity_lower = np.zeros(7)
        self.last_raw_velocity_upper = np.zeros(7)
        self.last_bound_conflicts = np.zeros(7, dtype=bool)
        self._mass = np.zeros((model.nv, model.nv))
        self._jacobian_a = np.zeros((3, model.nv))
        self._jacobian_b = np.zeros((3, model.nv))
        self._fromto = np.zeros(6)
        joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in spec.joint_names]
        self.joint_lower = np.asarray([model.jnt_range[index, 0] for index in joint_ids])
        self.joint_upper = np.asarray([model.jnt_range[index, 1] for index in joint_ids])

    def reset(self) -> None:
        self.previous_velocity[:] = 0.0
        self.previous_acceleration[:] = 0.0

    def _commit_velocity(self, velocity: np.ndarray) -> None:
        candidate = np.asarray(velocity, dtype=np.float64)
        self.previous_acceleration = (
            candidate - self.previous_velocity
        ) / self.config.task_period_s
        self.previous_velocity = candidate.copy()

    def reaction_velocity_map(self, data: mujoco.MjData) -> tuple[np.ndarray, float]:
        mujoco.mj_fullM(self.model, self._mass, data.qM)
        base = np.arange(self.base_dof_slice.start, self.base_dof_slice.stop)
        mass_bb = self._mass[np.ix_(base, base)]
        mass_ba = self._mass[np.ix_(base, self.dof_ids)]
        base_map = -np.linalg.solve(mass_bb, mass_ba)
        mapping = np.zeros((self.model.nv, 7))
        mapping[base, :] = base_map
        mapping[self.dof_ids, :] = np.eye(7)
        return mapping, float(np.linalg.norm(mass_bb @ base_map + mass_ba))

    def task_jacobians(self, data: mujoco.MjData, mapping: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        position = np.zeros((3, self.model.nv))
        rotation = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, data, position, rotation, self.flange_site)
        return position @ mapping, rotation @ mapping

    def _point_jacobian(self, data: mujoco.MjData, body: int, point: np.ndarray, output: np.ndarray) -> None:
        output.fill(0.0)
        if body != 0:
            mujoco.mj_jac(self.model, data, output, None, point, body)

    def clearance_gradient_for_pair(self, data: mujoco.MjData, pair: CollisionPair, mapping: np.ndarray) -> tuple[float, np.ndarray]:
        distance = float(mujoco.mj_geomDistance(self.model, data, pair.geom_a, pair.geom_b, self.config.clearance_query_max_m, self._fromto))
        first, second = self._fromto[:3].copy(), self._fromto[3:].copy()
        normal = (second - first) / max(float(np.linalg.norm(second - first)), 1e-12)
        body_a = int(self.model.geom_bodyid[pair.geom_a])
        body_b = int(self.model.geom_bodyid[pair.geom_b])

        # MuJoCo documents a geom1-to-geom2 segment, but in 3.3.2 its mesh-
        # primitive and mesh-mesh narrow phases return opposite point orders.
        # Form both exact point-Jacobian candidates and use one directional
        # distance probe to select the convention for the current query.
        self._point_jacobian(data, body_a, first, self._jacobian_a)
        self._point_jacobian(data, body_b, second, self._jacobian_b)
        first_is_a = np.asarray(normal @ (self._jacobian_b - self._jacobian_a) @ mapping)
        self._point_jacobian(data, body_a, second, self._jacobian_a)
        self._point_jacobian(data, body_b, first, self._jacobian_b)
        first_is_b = np.asarray(normal @ (self._jacobian_a - self._jacobian_b) @ mapping)
        probe = first_is_a if np.linalg.norm(first_is_a) >= np.linalg.norm(first_is_b) else first_is_b
        probe_norm = float(np.linalg.norm(probe))
        if probe_norm <= 1e-12 or distance >= self.config.clearance_query_max_m:
            return distance, np.zeros(7)
        probe = probe / probe_norm
        original_qpos = np.asarray(data.qpos).copy()
        epsilon = 1e-6
        mujoco.mj_integratePos(self.model, data.qpos, mapping @ probe, epsilon)
        mujoco.mj_forward(self.model, data)
        shifted = float(mujoco.mj_geomDistance(self.model, data, pair.geom_a, pair.geom_b, self.config.clearance_query_max_m, None))
        data.qpos[:] = original_qpos
        mujoco.mj_forward(self.model, data)
        directional = (shifted - distance) / epsilon
        candidate = min((first_is_a, first_is_b), key=lambda value: abs(float(value @ probe) - directional))
        return distance, candidate

    def _clearance_constraints(self, data: mujoco.MjData, mapping: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
        rows: list[np.ndarray] = []
        lower: list[float] = []
        minimum = float("inf")
        for pair in self.pairs:
            distance, gradient = self.clearance_gradient_for_pair(data, pair, mapping)
            minimum = min(minimum, distance)
            if distance <= self.config.clearance_activation_m and np.linalg.norm(gradient) > 1e-10:
                rows.append(gradient)
                lower.append(-self.config.clearance_barrier_gain * (distance - self.config.clearance_safe_m))
        return (np.vstack(rows) if rows else np.zeros((0, 7)), np.asarray(lower), minimum)

    def _bounds(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        cfg = self.config
        period = cfg.task_period_s
        speed = cfg.velocity_limit_scale * self.spec.velocity_limits_rad_s
        jerk_step = cfg.joint_jerk_limit_rad_s3 * period
        acceleration_lower = np.maximum(
            -self.spec.acceleration_limits_rad_s2,
            self.previous_acceleration - jerk_step,
        )
        acceleration_upper = np.minimum(
            self.spec.acceleration_limits_rad_s2,
            self.previous_acceleration + jerk_step,
        )
        lower = self.previous_velocity + acceleration_lower * period
        upper = self.previous_velocity + acceleration_upper * period
        distance_lower = q - self.joint_lower - cfg.joint_position_margin_rad
        distance_upper = self.joint_upper - cfg.joint_position_margin_rad - q
        for index in range(7):
            previous = float(self.previous_velocity[index])
            acceleration_limit = float(self.spec.acceleration_limits_rad_s2[index])

            def upper_speed_requirement(candidate: float) -> float:
                acceleration = (candidate - previous) / period
                return candidate + max(acceleration, 0.0) ** 2 / (
                    2.0 * cfg.joint_jerk_limit_rad_s3
                )

            if upper_speed_requirement(float(upper[index])) > float(speed[index]):
                safe, unsafe = float(lower[index]), float(upper[index])
                if upper_speed_requirement(safe) > float(speed[index]):
                    unsafe = safe
                else:
                    for _ in range(36):
                        midpoint = 0.5 * (safe + unsafe)
                        if upper_speed_requirement(midpoint) <= float(speed[index]):
                            safe = midpoint
                        else:
                            unsafe = midpoint
                upper[index] = safe

            def lower_speed_requirement(candidate: float) -> float:
                acceleration_toward = -(candidate - previous) / period
                return -candidate + max(acceleration_toward, 0.0) ** 2 / (
                    2.0 * cfg.joint_jerk_limit_rad_s3
                )

            if lower_speed_requirement(float(lower[index])) > float(speed[index]):
                unsafe, safe = float(lower[index]), float(upper[index])
                if lower_speed_requirement(safe) > float(speed[index]):
                    unsafe = safe
                else:
                    for _ in range(36):
                        midpoint = 0.5 * (unsafe + safe)
                        if lower_speed_requirement(midpoint) <= float(speed[index]):
                            safe = midpoint
                        else:
                            unsafe = midpoint
                lower[index] = safe

            def upper_stop_requirement(candidate: float) -> float:
                acceleration = (candidate - previous) / period
                travel = max(0.0, 0.5 * (previous + candidate) * period)
                return travel + jerk_limited_stopping_distance(
                    max(candidate, 0.0),
                    acceleration,
                    acceleration_limit,
                    cfg.joint_jerk_limit_rad_s3,
                )

            if upper_stop_requirement(float(upper[index])) > max(
                float(distance_upper[index]), 0.0
            ):
                safe, unsafe = float(lower[index]), float(upper[index])
                if upper_stop_requirement(safe) > max(
                    float(distance_upper[index]), 0.0
                ):
                    unsafe = safe
                else:
                    for _ in range(36):
                        midpoint = 0.5 * (safe + unsafe)
                        if upper_stop_requirement(midpoint) <= max(
                            float(distance_upper[index]), 0.0
                        ):
                            safe = midpoint
                        else:
                            unsafe = midpoint
                upper[index] = safe

            def lower_stop_requirement(candidate: float) -> float:
                acceleration_toward = -(candidate - previous) / period
                travel = max(0.0, -0.5 * (previous + candidate) * period)
                return travel + jerk_limited_stopping_distance(
                    max(-candidate, 0.0),
                    acceleration_toward,
                    acceleration_limit,
                    cfg.joint_jerk_limit_rad_s3,
                )

            if lower_stop_requirement(float(lower[index])) > max(
                float(distance_lower[index]), 0.0
            ):
                unsafe, safe = float(lower[index]), float(upper[index])
                if lower_stop_requirement(safe) > max(
                    float(distance_lower[index]), 0.0
                ):
                    unsafe = safe
                else:
                    for _ in range(36):
                        midpoint = 0.5 * (unsafe + safe)
                        if lower_stop_requirement(midpoint) <= max(
                            float(distance_lower[index]), 0.0
                        ):
                            safe = midpoint
                        else:
                            unsafe = midpoint
                lower[index] = safe
        invalid = lower > upper
        self.last_raw_velocity_lower = lower.copy()
        self.last_raw_velocity_upper = upper.copy()
        self.last_bound_conflicts = invalid.copy()
        if np.any(invalid):
            midpoint = 0.5 * (lower[invalid] + upper[invalid])
            lower[invalid] = midpoint
            upper[invalid] = midpoint
        self.last_velocity_lower = lower.copy()
        self.last_velocity_upper = upper.copy()
        return lower, upper

    def _solve_admm(self, hessian: np.ndarray, linear: np.ndarray, matrix: np.ndarray, lower: np.ndarray, upper: np.ndarray, initial: np.ndarray) -> tuple[np.ndarray, bool, str, int]:
        cfg = self.config
        system = hessian + cfg.admm_sigma * np.eye(7) + cfg.admm_rho * matrix.T @ matrix
        factor = cho_factor(system, lower=True, check_finite=False)
        value = initial.copy()
        product = matrix @ value
        auxiliary = np.minimum(np.maximum(product, lower), upper)
        dual = np.zeros(len(lower))
        status = "maximum_iterations"
        for iteration in range(1, cfg.qp_max_iterations + 1):
            rhs = cfg.admm_sigma * value - linear + matrix.T @ (cfg.admm_rho * auxiliary - dual)
            value = cho_solve(factor, rhs, check_finite=False)
            product = matrix @ value
            relaxed = cfg.admm_relaxation * product + (1.0 - cfg.admm_relaxation) * auxiliary
            auxiliary = np.minimum(np.maximum(relaxed + dual / cfg.admm_rho, lower), upper)
            dual += cfg.admm_rho * (relaxed - auxiliary)
            primal = float(np.max(np.abs(product - auxiliary)))
            dual_residual = float(np.max(np.abs(hessian @ value + linear + matrix.T @ dual)))
            if primal <= cfg.qp_tolerance * max(1.0, float(np.max(np.abs(product)))) and dual_residual <= 5.0 * cfg.qp_tolerance * max(1.0, float(np.max(np.abs(linear)))):
                status = "solved"
                break
        slack = np.minimum(product - lower, upper - product)
        feasible = bool(float(np.min(slack)) >= -cfg.feasibility_tolerance)
        if feasible and status != "solved":
            status = "solved_inaccurate"
        return value, feasible, status, iteration

    def solve(self, data: mujoco.MjData, *, target_position: np.ndarray, target_velocity: np.ndarray, target_rotation: np.ndarray, target_angular_velocity: np.ndarray | None = None) -> QPResult:
        started = time.perf_counter()
        mujoco.mj_forward(self.model, data)
        cfg = self.config
        mapping, reaction_residual = self.reaction_velocity_map(data)
        jacobian, rotation_jacobian = self.task_jacobians(data, mapping)
        position = np.asarray(data.site_xpos[self.flange_site]).copy()
        rotation = np.asarray(data.site_xmat[self.flange_site]).reshape(3, 3).copy()
        position_error = np.asarray(target_position) - position
        orientation_error = rotation_error_vector_world(target_rotation, rotation)
        velocity_command = _clip_norm(np.asarray(target_velocity) + cfg.position_gain * position_error, cfg.linear_speed_limit_m_s)
        angular_feedforward = np.zeros(3) if target_angular_velocity is None else np.asarray(target_angular_velocity)
        angular_command = smooth_saturate_norm(
            angular_feedforward + cfg.orientation_gain * orientation_error,
            cfg.angular_speed_limit_rad_s,
            cfg.angular_saturation_transition_ratio,
        )
        q = np.asarray(data.qpos[self.qpos_ids])
        posture_velocity = -cfg.posture_gain * (q - self.spec.home_joint_position)
        base_map = mapping[self.base_dof_slice, :]
        hessian = (
            cfg.position_weight * jacobian.T @ jacobian
            + cfg.orientation_weight * rotation_jacobian.T @ rotation_jacobian
            + cfg.posture_weight * np.eye(7)
            + cfg.base_reaction_weight * base_map.T @ base_map
        )
        linear = -(
            cfg.position_weight * jacobian.T @ velocity_command
            + cfg.orientation_weight * rotation_jacobian.T @ angular_command
            + cfg.posture_weight * posture_velocity
        )
        hessian = 0.5 * (hessian + hessian.T) + 1e-10 * np.eye(7)
        clearance_matrix, clearance_lower, minimum_clearance = self._clearance_constraints(data, mapping)
        lower, upper = self._bounds(q)
        matrix = np.vstack((clearance_matrix, np.eye(7)))
        constraint_lower = np.concatenate((clearance_lower, lower))
        constraint_upper = np.concatenate((np.full(len(clearance_lower), np.inf), upper))
        initial = np.clip(self.previous_velocity, lower, upper)
        solver_started = time.perf_counter()
        candidate, success, status, iterations = self._solve_admm(hessian, linear, matrix, constraint_lower, constraint_upper, initial)
        solver_latency = time.perf_counter() - solver_started
        all_slack = np.minimum(matrix @ candidate - constraint_lower, constraint_upper - matrix @ candidate)
        minimum_slack = float(np.min(all_slack))
        success = bool(success and minimum_slack >= -cfg.feasibility_tolerance)
        if not success:
            candidate = np.clip(self.previous_velocity, lower, upper)
        candidate = np.clip(candidate, lower, upper)
        binding = int(np.sum(clearance_matrix @ candidate - clearance_lower <= 2e-5)) if len(clearance_lower) else 0
        self._commit_velocity(candidate)
        return QPResult(
            candidate, success, status, iterations,
            time.perf_counter() - started, solver_latency,
            float(np.linalg.norm(position_error)), float(np.linalg.norm(orientation_error)),
            float(np.linalg.norm(jacobian @ candidate - velocity_command)),
            float(np.linalg.norm(rotation_jacobian @ candidate - angular_command)),
            float(minimum_clearance), int(len(clearance_lower)), binding,
            minimum_slack, reaction_residual,
        )
