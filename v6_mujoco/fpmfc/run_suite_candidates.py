"""Replay and aggregate top candidates for every seed in a PSO suite."""

from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from ..model import PROJECT_ROOT
from .config import DEFAULT_CONFIG_PATH
from .run_candidates import replay_optimizer_candidates
from .provenance import implementation_identity


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _dynamic_result_matches(
    summary_path: Path,
    *,
    optimizer_path: Path,
    config_path: Path,
    top_k: int,
) -> bool:
    try:
        payload = json.loads(summary_path.read_text(encoding="utf-8"))
        optimizer_payload = json.loads(optimizer_path.read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError):
        return False
    expected_count = min(
        int(top_k), len(optimizer_payload.get("optimizer", {}).get("top_candidates", []))
    )
    return bool(
        payload.get("optimizer_json_sha256") == _sha256(optimizer_path)
        and payload.get("config_sha256") == _sha256(config_path)
        and int(payload.get("candidate_count", -1)) == expected_count
        and payload.get("implementation_identity", {}).get("composite_sha256")
        == implementation_identity()["composite_sha256"]
    )


def replay_suite_candidates(
    suite_summary: Path,
    *,
    config_path: Path,
    output_root: Path,
    top_k: int = 5,
    parallelism: int = 1,
) -> dict[str, Any]:
    if top_k < 1 or parallelism < 1:
        raise ValueError("top_k and parallelism must be positive")
    suite_path = Path(suite_summary).resolve()
    config_file = Path(config_path).resolve()
    suite = json.loads(suite_path.read_text(encoding="utf-8"))
    root = Path(output_root).resolve()
    root.mkdir(parents=True, exist_ok=True)

    jobs: list[tuple[str, int, Path, Path]] = []
    result_paths: dict[tuple[str, int], Path] = {}
    for variant, variant_payload in suite["variants"].items():
        for run in variant_payload["runs"]:
            seed = int(run["seed"])
            optimizer_path = Path(run["path"]).resolve()
            run_root = root / variant / f"seed_{seed:02d}"
            summary = run_root / "candidate_replay_summary.json"
            result_paths[(variant, seed)] = summary
            if summary.exists() and _dynamic_result_matches(
                summary,
                optimizer_path=optimizer_path,
                config_path=config_file,
                top_k=top_k,
            ):
                print(
                    f"skip-dynamic variant={variant} seed={seed} path={summary}",
                    flush=True,
                )
                continue
            jobs.append((variant, seed, optimizer_path, run_root))

    def execute(job: tuple[str, int, Path, Path]) -> None:
        variant, seed, optimizer_path, run_root = job
        print(f"start-dynamic variant={variant} seed={seed}", flush=True)
        replay_optimizer_candidates(
            optimizer_path,
            config_path=config_file,
            output_root=run_root,
            top_k=top_k,
        )
        print(f"complete-dynamic variant={variant} seed={seed}", flush=True)

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

    variants: dict[str, Any] = {}
    for variant, variant_payload in suite["variants"].items():
        seed_reports: list[dict[str, Any]] = []
        all_ranked: list[dict[str, Any]] = []
        for run in variant_payload["runs"]:
            seed = int(run["seed"])
            summary_path = result_paths[(variant, seed)]
            replay = json.loads(summary_path.read_text(encoding="utf-8"))
            ranking = replay["dynamic_ranking"]
            top1_preserved = bool(
                ranking
                and int(ranking[0]["kinematic_rank"]) == 1
                and int(ranking[0]["dynamic_rank"]) == 1
            )
            seed_report = {
                "seed": seed,
                "summary_path": str(summary_path),
                "summary_sha256": _sha256(summary_path),
                "candidate_count": int(replay["candidate_count"]),
                "dynamic_accepted_candidate_count": int(
                    replay.get(
                        "dynamic_accepted_candidate_count",
                        replay["accepted_candidate_count"],
                    )
                ),
                "jointly_qualified_candidate_count": int(
                    replay.get(
                        "planning_and_dynamic_qualified_candidate_count",
                        replay["accepted_candidate_count"],
                    )
                ),
                "top1_preserved": top1_preserved,
                "best_dynamic_objective": (
                    None if not ranking else float(ranking[0]["dynamic_objective"])
                ),
            }
            seed_reports.append(seed_report)
            for row in ranking:
                all_ranked.append({"seed": seed, **row})

        qualified_seeds = [
            report
            for report in seed_reports
            if report["jointly_qualified_candidate_count"] > 0
        ]
        selected = (
            None
            if not all_ranked
            else min(all_ranked, key=lambda row: row["dynamic_objective"])
        )
        variants[variant] = {
            "seed_count": len(seed_reports),
            "qualified_seed_count": len(qualified_seeds),
            "qualified_seed_fraction": len(qualified_seeds) / len(seed_reports),
            "top1_preserved_count": sum(
                int(report["top1_preserved"]) for report in seed_reports
            ),
            "selected_dynamic_candidate": selected,
            "seeds": seed_reports,
        }

    all_seeds_qualified = all(
        variant["qualified_seed_count"] == variant["seed_count"]
        for variant in variants.values()
    )
    report = {
        "material_passport": {
            "origin_skill": "experiment-agent",
            "origin_mode": "run",
            "origin_date": datetime.now(timezone.utc).isoformat(),
            "verification_status": (
                "VERIFIED" if all_seeds_qualified else "PARTIALLY_VERIFIED"
            ),
            "version_label": "dynamic_suite_summary_v1",
        },
        "suite_summary": str(suite_path),
        "suite_summary_sha256": _sha256(suite_path),
        "effective_config_sha256": suite.get("effective_config_sha256"),
        "config_path": str(config_file),
        "config_sha256": _sha256(config_file),
        "implementation_identity": implementation_identity(),
        "top_k": int(top_k),
        "variants": variants,
    }
    output = root / "dynamic_suite_summary.json"
    output.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite-summary", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--top-k", type=int, default=5)
    parser.add_argument("--parallel", type=int, default=1)
    parser.add_argument(
        "--output-root",
        type=Path,
        default=PROJECT_ROOT / "output" / "fpmfc" / "precontact" / "formal_suite",
    )
    args = parser.parse_args()
    report = replay_suite_candidates(
        args.suite_summary,
        config_path=args.config,
        output_root=args.output_root,
        top_k=args.top_k,
        parallelism=args.parallel,
    )
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
