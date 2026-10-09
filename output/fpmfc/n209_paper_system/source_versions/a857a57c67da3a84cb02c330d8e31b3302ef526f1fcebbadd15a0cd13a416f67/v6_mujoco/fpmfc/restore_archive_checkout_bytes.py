"""Restore frozen source bytes in a clean Windows worktree after CRLF checkout.

This does not change Git content or historical experiment artifacts. It refuses
any file whose only difference from the HEAD blob is not CRLF conversion.
"""

from __future__ import annotations

import json
import subprocess
from pathlib import Path

from ..model import PROJECT_ROOT
from .audit_contact import CONTACTS, SOURCE
from .contact_model import default_contact_model_spec


def main() -> None:
    source = json.loads((PROJECT_ROOT / SOURCE / "metrics.json").read_text(encoding="utf-8"))
    contact = json.loads((PROJECT_ROOT / CONTACTS["admittance"] / "metrics.json").read_text(encoding="utf-8"))
    paths = set(source["implementation_identity"]["files"])
    paths.update(contact["implementation_identity"]["files"])
    paths.add(contact["precontact_config_path"].replace("\\", "/"))
    paths.add(default_contact_model_spec().source_urdf.relative_to(PROJECT_ROOT).as_posix())
    for relative in sorted(paths):
        path = PROJECT_ROOT / relative
        blob = subprocess.check_output(["git", "show", f"HEAD:{relative}"], cwd=PROJECT_ROOT)
        current = path.read_bytes()
        if current == blob:
            continue
        if current.replace(b"\r\n", b"\n") != blob:
            raise RuntimeError(f"refusing to overwrite non-CRLF difference: {relative}")
        path.write_bytes(blob)
        print(relative)


if __name__ == "__main__":
    main()
