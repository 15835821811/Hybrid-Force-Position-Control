"""Composite identities for N110D execution and independent checking."""

from __future__ import annotations

import hashlib
import json

from ..model import PROJECT_ROOT
from .handoff_provenance import n110c_implementation_identity


RUNTIME_PATHS = (
    "v6_mujoco/fpmfc/force_regulation_outer.py",
    "v6_mujoco/fpmfc/force_regulation_contract.py",
    "v6_mujoco/fpmfc/run_force_regulation.py",
    "v6_mujoco/fpmfc/force_regulation_provenance.py",
)
VERIFICATION_PATHS = (
    "v6_mujoco/fpmfc/force_regulation_analysis.py",
    "v6_mujoco/fpmfc/validate_force_regulation.py",
)


def _identity(paths: tuple[str, ...], schema: str, parent: dict) -> dict:
    files = {path: hashlib.sha256((PROJECT_ROOT/path).read_bytes()).hexdigest() for path in sorted(paths)}
    result = {"schema_version": schema, "parent": parent, "source_files": files}
    encoded = json.dumps(result, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    result["composite_sha256"] = hashlib.sha256(encoded).hexdigest()
    return result


def runtime_identity() -> dict:
    return _identity(RUNTIME_PATHS, "n110d_force_regulation_runtime_v1", n110c_implementation_identity())


def verification_identity() -> dict:
    return _identity(VERIFICATION_PATHS, "n110d_force_regulation_verification_v1",
                     {"runtime_composite_sha256": runtime_identity()["composite_sha256"]})
