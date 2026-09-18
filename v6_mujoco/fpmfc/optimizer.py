"""Deterministic bounded PSO for paper-faithful capture time and arm shape."""

from __future__ import annotations

import argparse
import hashlib
import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Mapping

import numpy as np

from ..model import PROJECT_ROOT, default_model_spec
from .config import DEFAULT_CONFIG_PATH, apply_runtime_overrides, load_fpmfc_config
from .provenance import implementation_identity
from .rollout import PrecontactRolloutEvaluator, PrecontactRolloutResult


@dataclass(frozen=True)
class PSOResult:
    seed: int
    population: int
    generations_requested: int
    generations_completed: int
    converged: bool
    best_position: np.ndarray
    best_objective: float
    best_rollout: PrecontactRolloutResult
    history: np.ndarray
    evaluations: int
    top_positions: np.ndarray
    top_objectives: np.ndarray
    top_rollouts: tuple[PrecontactRolloutResult, ...]

    def to_dict(self) -> dict[str, Any]:
        return {
            "seed": self.seed,
            "population": self.population,
            "generations_requested": self.generations_requested,
            "generations_completed": self.generations_completed,
            "converged": self.converged,
            "best_position": self.best_position.tolist(),
            "best_objective": self.best_objective,
            "best_rollout": self.best_rollout.to_dict(),
            "history": self.history.tolist(),
            "evaluations": self.evaluations,
            "top_candidates": [
                {
                    "position": position.tolist(),
                    "objective": float(objective),
                    "rollout": rollout.to_dict(),
                }
                for position, objective, rollout in zip(
                    self.top_positions, self.top_objectives, self.top_rollouts
                )
            ],
        }


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _effective_config_sha256(config: Mapping[str, Any]) -> str:
    encoded = json.dumps(
        config, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def particle_swarm_optimize(
    evaluate: Callable[[float, float], PrecontactRolloutResult],
    bounds: np.ndarray,
    *,
    seed: int,
    population: int,
    generations: int,
    inertia: float,
    cognitive: float,
    social: float,
    velocity_fraction: float,
    convergence_tolerance: float,
    stagnation_generations: int,
    progress: Callable[[int, PrecontactRolloutResult], None] | None = None,
) -> PSOResult:
    bounds_value = np.asarray(bounds, dtype=np.float64)
    if (
        bounds_value.shape != (2, 2)
        or np.any(bounds_value[:, 0] > bounds_value[:, 1])
        or np.all(bounds_value[:, 0] == bounds_value[:, 1])
    ):
        raise ValueError(
            "PSO bounds must have shape (2, 2), nondecreasing rows, and one free variable"
        )
    if population < 2 or generations < 1 or stagnation_generations < 1:
        raise ValueError("invalid PSO population, generation, or stagnation count")
    rng = np.random.default_rng(int(seed))
    lower = bounds_value[:, 0]
    upper = bounds_value[:, 1]
    span = upper - lower
    positions = rng.uniform(lower, upper, size=(population, 2))
    velocity_limit = float(velocity_fraction) * span
    velocities = rng.uniform(-velocity_limit, velocity_limit, size=(population, 2))
    personal_positions = positions.copy()
    personal_values = np.full(population, np.inf)
    personal_results: list[PrecontactRolloutResult | None] = [None] * population
    global_position = positions[0].copy()
    global_value = float("inf")
    global_result: PrecontactRolloutResult | None = None
    history: list[float] = []
    stable_count = 0
    evaluations = 0
    converged = False

    for generation in range(generations):
        previous_best = global_value
        for index in range(population):
            result = evaluate(float(positions[index, 0]), float(positions[index, 1]))
            evaluations += 1
            value = float(result.objective)
            if value < personal_values[index]:
                personal_values[index] = value
                personal_positions[index] = positions[index].copy()
                personal_results[index] = result
            if value < global_value:
                global_value = value
                global_position = positions[index].copy()
                global_result = result
        if global_result is None:
            raise RuntimeError("PSO did not evaluate a finite candidate")
        history.append(global_value)
        if progress is not None:
            progress(generation + 1, global_result)
        if np.isfinite(previous_best) and abs(previous_best - global_value) <= convergence_tolerance:
            stable_count += 1
        else:
            stable_count = 0
        if stable_count >= stagnation_generations:
            converged = True
            break
        random_personal = rng.random((population, 2))
        random_global = rng.random((population, 2))
        velocities = (
            inertia * velocities
            + cognitive * random_personal * (personal_positions - positions)
            + social * random_global * (global_position[None, :] - positions)
        )
        velocities = np.clip(velocities, -velocity_limit, velocity_limit)
        positions = np.clip(positions + velocities, lower, upper)

    if global_result is None:
        raise RuntimeError("PSO produced no result")
    top_indices = np.argsort(personal_values)[: min(5, population)]
    if any(personal_results[int(index)] is None for index in top_indices):
        raise RuntimeError("PSO top-candidate bookkeeping is incomplete")
    return PSOResult(
        seed=int(seed),
        population=int(population),
        generations_requested=int(generations),
        generations_completed=len(history),
        converged=converged,
        best_position=global_position,
        best_objective=global_value,
        best_rollout=global_result,
        history=np.asarray(history, dtype=np.float64),
        evaluations=evaluations,
        top_positions=personal_positions[top_indices].copy(),
        top_objectives=personal_values[top_indices].copy(),
        top_rollouts=tuple(
            personal_results[int(index)] for index in top_indices  # type: ignore[misc]
        ),
    )


def optimize_from_config(
    config: Mapping[str, Any],
    *,
    seed: int,
    population: int | None = None,
    generations: int | None = None,
    fixed_arm_angle_rad: float | None = None,
    progress: Callable[[int, PrecontactRolloutResult], None] | None = None,
) -> PSOResult:
    optimizer = config["optimization"]
    time_bounds = config["trajectory"]["capture_time_bounds_s"]
    arm_bounds = optimizer["terminal_arm_angle_bounds_rad"]
    if fixed_arm_angle_rad is not None:
        fixed_angle = float(fixed_arm_angle_rad)
        if not float(arm_bounds[0]) <= fixed_angle <= float(arm_bounds[1]):
            raise ValueError("fixed arm angle lies outside configured bounds")
        arm_bounds = [fixed_angle, fixed_angle]
    evaluator = PrecontactRolloutEvaluator(config)
    return particle_swarm_optimize(
        evaluator.evaluate,
        np.asarray([time_bounds, arm_bounds], dtype=np.float64),
        seed=seed,
        population=int(optimizer["pso_population"] if population is None else population),
        generations=int(optimizer["pso_generations"] if generations is None else generations),
        inertia=float(optimizer["pso_inertia"]),
        cognitive=float(optimizer["pso_cognitive"]),
        social=float(optimizer["pso_social"]),
        velocity_fraction=float(optimizer["pso_velocity_fraction"]),
        convergence_tolerance=float(optimizer["convergence_tolerance"]),
        stagnation_generations=int(optimizer["stagnation_generations"]),
        progress=progress,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--population", type=int)
    parser.add_argument("--generations", type=int)
    parser.add_argument(
        "--fixed-arm-angle",
        type=float,
        help="hold psi exactly fixed and optimize capture time only",
    )
    parser.add_argument(
        "--objective-weights",
        nargs=2,
        type=float,
        metavar=("BASE", "ALIGNMENT"),
        help="override the normalized objective weights for sensitivity runs",
    )
    parser.add_argument(
        "--planning-clearance",
        type=float,
        metavar="METERS",
        help="override the optimization/controller clearance while preserving the source acceptance threshold",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_ROOT / "output" / "fpmfc" / "optimization" / "pso_seed_00.json",
    )
    parser.add_argument(
        "--progress-output",
        type=Path,
        help="JSONL generation log; defaults to the output filename with .jsonl suffix",
    )
    args = parser.parse_args()
    config = apply_runtime_overrides(
        load_fpmfc_config(args.config),
        objective_weights=args.objective_weights,
        planning_clearance_m=args.planning_clearance,
    )
    code_identity = implementation_identity()
    output = Path(args.output).resolve()
    progress_output = (
        output.with_suffix(".jsonl")
        if args.progress_output is None
        else Path(args.progress_output).resolve()
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    progress_output.parent.mkdir(parents=True, exist_ok=True)
    progress_output.write_text(
        json.dumps(
            {
                "event": "start",
                "seed": args.seed,
                "population": args.population or config["optimization"]["pso_population"],
                "generations": args.generations or config["optimization"]["pso_generations"],
                "fixed_arm_angle_rad": args.fixed_arm_angle,
                "objective_weights": config["optimization"]["objective_weights"],
                "planning_clearance_m": config["controller"]["minimum_clearance_m"],
                "acceptance_clearance_m": config["acceptance"].get(
                    "minimum_clearance_m", config["controller"]["minimum_clearance_m"]
                ),
                "effective_config_sha256": _effective_config_sha256(config),
                "implementation_identity_sha256": code_identity["composite_sha256"],
                "config_path": str(Path(args.config).resolve()),
                "started_at": datetime.now(timezone.utc).isoformat(),
            },
            ensure_ascii=False,
        )
        + "\n",
        encoding="utf-8",
    )

    def report(generation: int, rollout: PrecontactRolloutResult) -> None:
        print(
            f"generation={generation} objective={rollout.objective:.9g} "
            f"T={rollout.capture_time_s:.6f} psi={rollout.terminal_arm_angle_rad:.6f} "
            f"feasible={rollout.feasible}",
            flush=True,
        )
        with progress_output.open("a", encoding="utf-8") as handle:
            handle.write(
                json.dumps(
                    {
                        "event": "generation",
                        "generation": generation,
                        "objective": rollout.objective,
                        "capture_time_s": rollout.capture_time_s,
                        "terminal_arm_angle_rad": rollout.terminal_arm_angle_rad,
                        "feasible": rollout.feasible,
                        "timestamp": datetime.now(timezone.utc).isoformat(),
                    },
                    ensure_ascii=False,
                )
                + "\n"
            )

    result = optimize_from_config(
        config,
        seed=args.seed,
        population=args.population,
        generations=args.generations,
        fixed_arm_angle_rad=args.fixed_arm_angle,
        progress=report,
    )
    payload = {
        "material_passport": {
            "origin_skill": "experiment-agent",
            "origin_mode": "run",
            "origin_date": datetime.now(timezone.utc).isoformat(),
            "verification_status": "UNVERIFIED",
            "version_label": "exp_result_v1",
        },
        "experiment_id": config["experiment_id"],
        "config_path": str(Path(args.config).resolve()),
        "config_sha256": _sha256(Path(args.config).resolve()),
        "effective_config_sha256": _effective_config_sha256(config),
        "effective_config": config,
        "implementation_identity": code_identity,
        "model_identity": default_model_spec().identity(),
        "progress_path": str(progress_output),
        "fixed_arm_angle_rad": args.fixed_arm_angle,
        "runtime_overrides": {
            "objective_weights": args.objective_weights,
            "planning_clearance_m": args.planning_clearance,
        },
        "optimizer": result.to_dict(),
    }
    output.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    with progress_output.open("a", encoding="utf-8") as handle:
        handle.write(
            json.dumps(
                {
                    "event": "complete",
                    "generations_completed": result.generations_completed,
                    "evaluations": result.evaluations,
                    "best_objective": result.best_objective,
                    "completed_at": datetime.now(timezone.utc).isoformat(),
                    "output_path": str(output),
                },
                ensure_ascii=False,
            )
            + "\n"
        )
    print(f"wrote={output.resolve()}", flush=True)


if __name__ == "__main__":
    main()
