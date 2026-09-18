"""Run and aggregate multi-seed joint and fixed-shape PSO variants."""

from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from ..model import PROJECT_ROOT
from .config import DEFAULT_CONFIG_PATH, apply_runtime_overrides, load_fpmfc_config
from .provenance import implementation_identity


VARIANT_FIXED_ANGLE: dict[str, float | None] = {
    "joint": None,
    "fixed0": 0.0,
    "fixedpi2": float(np.pi / 2.0),
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _effective_config_sha256(config: dict[str, Any]) -> str:
    encoded = json.dumps(
        config, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _result_matches(
    path: Path,
    *,
    config_sha256: str,
    effective_config_sha256: str,
    seed: int,
    population: int,
    generations: int,
    fixed_angle: float | None,
    implementation_sha256: str | None = None,
) -> bool:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
        optimizer = payload["optimizer"]
    except (OSError, ValueError, KeyError, TypeError):
        return False
    stored_fixed = payload.get("fixed_arm_angle_rad")
    fixed_matches = (
        stored_fixed is None
        if fixed_angle is None
        else stored_fixed is not None
        and abs(float(stored_fixed) - fixed_angle) <= 1e-12
    )
    return bool(
        payload.get("config_sha256") == config_sha256
        and payload.get("effective_config_sha256") == effective_config_sha256
        and int(optimizer["seed"]) == seed
        and int(optimizer["population"]) == population
        and int(optimizer["generations_requested"]) == generations
        and fixed_matches
        and (
            implementation_sha256 is None
            or payload.get("implementation_identity", {}).get("composite_sha256")
            == implementation_sha256
        )
    )


def aggregate_suite(output_root: Path, variants: list[str], seeds: list[int]) -> dict[str, Any]:
    root = Path(output_root).resolve()
    variant_reports: dict[str, Any] = {}
    for variant in variants:
        runs: list[dict[str, Any]] = []
        for seed in seeds:
            path = root / variant / f"pso_seed_{seed:02d}.json"
            payload = json.loads(path.read_text(encoding="utf-8"))
            optimizer = payload["optimizer"]
            best = optimizer["best_rollout"]
            runs.append(
                {
                    "seed": seed,
                    "path": str(path),
                    "sha256": _sha256(path),
                    "effective_config_sha256": payload.get(
                        "effective_config_sha256"
                    ),
                    "runtime_overrides": payload.get("runtime_overrides") or {},
                    "implementation_identity_sha256": payload.get(
                        "implementation_identity", {}
                    ).get("composite_sha256"),
                    "best_objective": float(optimizer["best_objective"]),
                    "capture_time_s": float(optimizer["best_position"][0]),
                    "terminal_arm_angle_rad": float(optimizer["best_position"][1]),
                    "feasible": bool(best["feasible"]),
                    "generations_completed": int(optimizer["generations_completed"]),
                    "evaluations": int(optimizer["evaluations"]),
                    "maximum_base_angular_velocity_rad_s": float(
                        best["maximum_base_angular_velocity_rad_s"]
                    ),
                    "alignment_angle_rad": float(best["alignment_angle_rad"]),
                    "minimum_clearance_m": best["minimum_clearance_m"],
                }
            )
        objectives = np.asarray([run["best_objective"] for run in runs])
        feasible_runs = [run for run in runs if run["feasible"]]
        best_run = min(feasible_runs or runs, key=lambda run: run["best_objective"])
        variant_reports[variant] = {
            "fixed_arm_angle_rad": VARIANT_FIXED_ANGLE[variant],
            "seed_count": len(runs),
            "feasible_seed_count": len(feasible_runs),
            "feasible_seed_fraction": len(feasible_runs) / len(runs),
            "best_objective_mean": float(np.mean(objectives)),
            "best_objective_std": float(np.std(objectives, ddof=1))
            if len(objectives) > 1
            else 0.0,
            "best_objective_median": float(np.median(objectives)),
            "best_objective_min": float(np.min(objectives)),
            "best_objective_max": float(np.max(objectives)),
            "selected_seed": int(best_run["seed"]),
            "selected_result_path": best_run["path"],
            "runs": runs,
        }
    effective_hashes = {
        run["effective_config_sha256"]
        for variant in variant_reports.values()
        for run in variant["runs"]
    }
    if None in effective_hashes or len(effective_hashes) != 1:
        raise ValueError(
            "suite aggregation requires one common, non-missing effective config hash"
        )
    implementation_hashes = {
        run["implementation_identity_sha256"]
        for variant in variant_reports.values()
        for run in variant["runs"]
    }
    if None in implementation_hashes or len(implementation_hashes) != 1:
        raise ValueError(
            "suite aggregation requires one common, non-missing implementation hash"
        )
    report = {
        "material_passport": {
            "origin_skill": "experiment-agent",
            "origin_mode": "run",
            "origin_date": datetime.now(timezone.utc).isoformat(),
            "verification_status": "UNVERIFIED_DYNAMIC",
            "version_label": "pso_suite_v1",
        },
        "effective_config_sha256": next(iter(effective_hashes)),
        "implementation_identity_sha256": next(iter(implementation_hashes)),
        "variants": variant_reports,
    }
    summary_path = root / "pso_suite_summary.json"
    summary_path.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    return report


def run_suite(
    *,
    config_path: Path,
    output_root: Path,
    variants: list[str],
    seeds: list[int],
    population: int,
    generations: int,
    overwrite: bool,
    objective_weights: tuple[float, float] | None = None,
    planning_clearance_m: float | None = None,
    parallelism: int = 1,
) -> dict[str, Any]:
    if parallelism < 1:
        raise ValueError("parallelism must be positive")
    config_file = Path(config_path).resolve()
    config_sha256 = _sha256(config_file)
    effective_config = apply_runtime_overrides(
        load_fpmfc_config(config_file),
        objective_weights=objective_weights,
        planning_clearance_m=planning_clearance_m,
    )
    effective_config_sha256 = _effective_config_sha256(effective_config)
    code_identity = implementation_identity()
    root = Path(output_root).resolve()
    jobs: list[tuple[str, int, list[str], Path]] = []
    for variant in variants:
        fixed_angle = VARIANT_FIXED_ANGLE[variant]
        variant_dir = root / variant
        variant_dir.mkdir(parents=True, exist_ok=True)
        for seed in seeds:
            output = variant_dir / f"pso_seed_{seed:02d}.json"
            if output.exists() and not overwrite:
                if _result_matches(
                    output,
                    config_sha256=config_sha256,
                    effective_config_sha256=effective_config_sha256,
                    seed=seed,
                    population=population,
                    generations=generations,
                    fixed_angle=fixed_angle,
                    implementation_sha256=code_identity["composite_sha256"],
                ):
                    print(f"skip-complete variant={variant} seed={seed} path={output}", flush=True)
                    continue
                raise FileExistsError(
                    f"existing result does not match this suite: {output}; "
                    "choose another output root or pass --overwrite"
                )
            command = [
                sys.executable,
                "-m",
                "v6_mujoco.fpmfc.optimizer",
                "--config",
                str(config_file),
                "--seed",
                str(seed),
                "--population",
                str(population),
                "--generations",
                str(generations),
                "--output",
                str(output),
            ]
            if fixed_angle is not None:
                command.extend(("--fixed-arm-angle", repr(fixed_angle)))
            if objective_weights is not None:
                command.extend(
                    (
                        "--objective-weights",
                        repr(objective_weights[0]),
                        repr(objective_weights[1]),
                    )
                )
            if planning_clearance_m is not None:
                command.extend(("--planning-clearance", repr(planning_clearance_m)))
            jobs.append((variant, seed, command, output))

    def execute(job: tuple[str, int, list[str], Path]) -> None:
        variant, seed, command, output = job
        print(f"start variant={variant} seed={seed} output={output}", flush=True)
        subprocess.run(command, cwd=PROJECT_ROOT, check=True)
        print(f"complete variant={variant} seed={seed} output={output}", flush=True)

    if parallelism == 1:
        for job in jobs:
            execute(job)
    else:
        with concurrent.futures.ThreadPoolExecutor(
            max_workers=min(parallelism, max(1, len(jobs)))
        ) as executor:
            futures = [executor.submit(execute, job) for job in jobs]
            for future in concurrent.futures.as_completed(futures):
                future.result()
    return aggregate_suite(root, variants, seeds)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument(
        "--output-root",
        type=Path,
        default=PROJECT_ROOT / "output" / "fpmfc" / "optimization" / "formal",
    )
    parser.add_argument(
        "--variants",
        nargs="+",
        choices=tuple(VARIANT_FIXED_ANGLE),
        default=["joint"],
    )
    parser.add_argument("--seeds", nargs="+", type=int)
    parser.add_argument("--population", type=int)
    parser.add_argument("--generations", type=int)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument(
        "--parallel",
        type=int,
        default=1,
        metavar="N",
        help="run up to N independent seed/variant optimizer processes concurrently",
    )
    parser.add_argument(
        "--objective-weights",
        nargs=2,
        type=float,
        metavar=("BASE", "ALIGNMENT"),
    )
    parser.add_argument(
        "--planning-clearance",
        type=float,
        metavar="METERS",
    )
    args = parser.parse_args()
    config = load_fpmfc_config(args.config)
    optimizer = config["optimization"]
    seeds = list(map(int, optimizer["seeds"] if args.seeds is None else args.seeds))
    if len(set(seeds)) != len(seeds):
        raise ValueError("suite seeds must be unique")
    variants = list(dict.fromkeys(args.variants))
    objective_weights = (
        None
        if args.objective_weights is None
        else (float(args.objective_weights[0]), float(args.objective_weights[1]))
    )
    if objective_weights is not None and (
        any(value < 0.0 for value in objective_weights)
        or not any(value > 0.0 for value in objective_weights)
    ):
        raise ValueError("objective weights must be nonnegative and not both zero")
    report = run_suite(
        config_path=args.config,
        output_root=args.output_root,
        variants=variants,
        seeds=seeds,
        population=int(optimizer["pso_population"] if args.population is None else args.population),
        generations=int(
            optimizer["pso_generations"] if args.generations is None else args.generations
        ),
        overwrite=args.overwrite,
        objective_weights=objective_weights,
        planning_clearance_m=args.planning_clearance,
        parallelism=args.parallel,
    )
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
