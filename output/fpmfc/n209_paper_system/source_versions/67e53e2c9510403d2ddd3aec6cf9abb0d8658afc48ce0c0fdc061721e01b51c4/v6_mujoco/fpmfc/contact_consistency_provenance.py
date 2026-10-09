"""Versioned implementation identity for new N110 contact observations/runs."""

from __future__ import annotations

import hashlib
import json

from ..model import PROJECT_ROOT
from .contact_provenance import contact_implementation_identity
from .provenance import implementation_identity


N110_RUNTIME_PATHS = (
    "v6_mujoco/fpmfc/contact_observation.py",
    "v6_mujoco/fpmfc/contact_consistency_metrics.py",
    "v6_mujoco/fpmfc/contact_affine_controller.py",
    "v6_mujoco/fpmfc/run_contact_consistent.py",
    "v6_mujoco/fpmfc/contact_consistency_provenance.py",
)
N110_VERIFICATION_PATHS = (
    "v6_mujoco/fpmfc/audit_contact.py",
    "v6_mujoco/fpmfc/reevaluate_contact.py",
    "v6_mujoco/fpmfc/validate_contact_consistent.py",
)


def n110_implementation_identity() -> dict[str, object]:
    files = {
        relative: hashlib.sha256((PROJECT_ROOT / relative).read_bytes()).hexdigest()
        for relative in sorted(N110_RUNTIME_PATHS)
    }
    payload = {
        "schema_version": "n110_contact_consistency_implementation_v1",
        "frozen_precontact_identity": implementation_identity(),
        "frozen_contact_identity": contact_implementation_identity(),
        "new_runtime_source_files": files,
    }
    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    payload["composite_sha256"] = hashlib.sha256(canonical).hexdigest()
    return payload


def n110_verification_identity() -> dict[str, object]:
    files = {
        relative: hashlib.sha256((PROJECT_ROOT / relative).read_bytes()).hexdigest()
        for relative in sorted(N110_VERIFICATION_PATHS)
    }
    payload = {
        "schema_version": "n110_contact_consistency_verification_v1",
        "runtime_composite_sha256": n110_implementation_identity()["composite_sha256"],
        "verification_source_files": files,
    }
    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    payload["composite_sha256"] = hashlib.sha256(canonical).hexdigest()
    return payload
