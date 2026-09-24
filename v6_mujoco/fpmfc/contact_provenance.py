"""Implementation identity for contact-extension experiments."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

from ..model import PROJECT_ROOT


CONTACT_IMPLEMENTATION_PATHS = (
    "configs/fpmfc_contact.yaml",
    "models/flexiv_rizon4s_contact_scene.xml",
    "v6_mujoco/fpmfc/contact.py",
    "v6_mujoco/fpmfc/contact_config.py",
    "v6_mujoco/fpmfc/contact_model.py",
    "v6_mujoco/fpmfc/contact_precheck.py",
    "v6_mujoco/fpmfc/run_contact.py",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def contact_implementation_identity() -> dict[str, Any]:
    files = {
        relative: _sha256(PROJECT_ROOT / relative)
        for relative in sorted(CONTACT_IMPLEMENTATION_PATHS)
    }
    encoded = json.dumps(
        files, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    return {
        "schema_version": "fpmfc_contact_implementation_identity_v1",
        "files": files,
        "composite_sha256": hashlib.sha256(encoded).hexdigest(),
    }
