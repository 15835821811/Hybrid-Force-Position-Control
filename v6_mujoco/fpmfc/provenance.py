"""Hash the complete implementation used by formal FPMFC experiments."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

from ..model import PROJECT_ROOT


IMPLEMENTATION_PATHS = (
    "models/flexiv_rizon4s_scene.xml",
    "v6_mujoco/collision.py",
    "v6_mujoco/hierarchical_qp.py",
    "v6_mujoco/model.py",
    "v6_mujoco/run.py",
    "v6_mujoco/fpmfc/config.py",
    "v6_mujoco/fpmfc/controller.py",
    "v6_mujoco/fpmfc/dynamics.py",
    "v6_mujoco/fpmfc/optimizer.py",
    "v6_mujoco/fpmfc/provenance.py",
    "v6_mujoco/fpmfc/pso_suite.py",
    "v6_mujoco/fpmfc/rollout.py",
    "v6_mujoco/fpmfc/run_candidates.py",
    "v6_mujoco/fpmfc/run_capture.py",
    "v6_mujoco/fpmfc/run_suite_candidates.py",
    "v6_mujoco/fpmfc/shape.py",
    "v6_mujoco/fpmfc/target.py",
    "v6_mujoco/fpmfc/trajectory.py",
    "v6_mujoco/fpmfc/validate_capture.py",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def implementation_identity() -> dict[str, Any]:
    """Return per-file and canonical composite hashes for the frozen pipeline."""

    files = {
        relative: _sha256(PROJECT_ROOT / relative)
        for relative in sorted(IMPLEMENTATION_PATHS)
    }
    encoded = json.dumps(
        files, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    return {
        "schema_version": "fpmfc_implementation_identity_v1",
        "files": files,
        "composite_sha256": hashlib.sha256(encoded).hexdigest(),
    }
