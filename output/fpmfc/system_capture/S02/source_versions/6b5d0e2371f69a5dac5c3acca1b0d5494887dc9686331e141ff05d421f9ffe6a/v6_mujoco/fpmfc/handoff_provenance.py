"""N110C runtime and independent-verification source identities."""

from __future__ import annotations

import hashlib
import json

from ..model import PROJECT_ROOT
from .contact_consistency_provenance import n110_implementation_identity


RUNTIME_PATHS = (
    "v6_mujoco/fpmfc/handoff_trajectory.py",
    "v6_mujoco/fpmfc/handoff_contract.py",
    "v6_mujoco/fpmfc/run_handoff_precontact.py",
    "v6_mujoco/fpmfc/run_handoff_contact.py",
    "v6_mujoco/fpmfc/handoff_provenance.py",
)
VERIFICATION_PATHS = (
    "v6_mujoco/fpmfc/validate_handoff_precontact.py",
    "v6_mujoco/fpmfc/validate_handoff_contact.py",
)


def _identity(paths: tuple[str, ...], schema: str, parent: dict[str, object]) -> dict[str, object]:
    files = {
        path: hashlib.sha256((PROJECT_ROOT/path).read_bytes()).hexdigest()
        for path in sorted(paths)
    }
    payload = {"schema_version": schema, "parent": parent, "source_files": files}
    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    payload["composite_sha256"] = hashlib.sha256(canonical).hexdigest()
    return payload


def n110c_implementation_identity() -> dict[str, object]:
    return _identity(RUNTIME_PATHS, "n110c_handoff_runtime_v1", n110_implementation_identity())


def n110c_verification_identity() -> dict[str, object]:
    return _identity(
        VERIFICATION_PATHS, "n110c_handoff_verification_v1",
        {"runtime_composite_sha256": n110c_implementation_identity()["composite_sha256"]},
    )
