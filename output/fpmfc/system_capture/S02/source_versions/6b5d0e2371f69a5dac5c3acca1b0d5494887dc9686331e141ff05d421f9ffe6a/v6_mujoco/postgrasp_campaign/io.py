"""Atomic artifacts, immutable stage identities, and explicit resource accounting."""
import hashlib
import json
import os
from pathlib import Path
import time

import numpy as np
from v6_mujoco.model import PROJECT_ROOT
from v6_mujoco.postgrasp.physics import digest

ROOT = PROJECT_ROOT / "output/fpmfc/postgrasp_campaign"
OLD = PROJECT_ROOT / "output/fpmfc/n200_postgrasp"
CAL = PROJECT_ROOT / "output/fpmfc/n201_interface_calibration"
CONFIG = PROJECT_ROOT / "configs/postgrasp_campaign.yaml"
MODEL = PROJECT_ROOT / "models/flexiv_rizon4s_postgrasp_campaign.xml"


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def save(path, payload):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="\n") as stream:
        json.dump(payload, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write("\n"); stream.flush(); os.fsync(stream.fileno())
    os.replace(temporary, path)


def save_npz(path, arrays):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("wb") as stream:
        np.savez_compressed(stream, **arrays)
        stream.flush(); os.fsync(stream.fileno())
    os.replace(temporary, path)


def identity(paths):
    return {Path(p).relative_to(PROJECT_ROOT).as_posix(): digest(Path(p)) for p in paths}


def check_identity(values):
    for relative, expected in values.items():
        path = PROJECT_ROOT / relative
        if not path.exists():
            raise RuntimeError(f"MISSING_SOURCE: {relative}")
        if digest(path) != expected:
            raise RuntimeError(f"MODEL_IDENTITY_MISMATCH: {relative}")


def design():
    manifest = read(ROOT / "campaign_manifest.json")
    if digest(CONFIG) != manifest["config_sha256"]:
        raise RuntimeError("MODEL_IDENTITY_MISMATCH: preregistered config changed")
    if digest(ROOT / "source_identity.json") != manifest["source_identity_sha256"]:
        raise RuntimeError("MODEL_IDENTITY_MISMATCH: source identity manifest changed")
    if digest(ROOT / "restricted_fixture/input_contract.json") != manifest["restricted_fixture_contract_sha256"]:
        raise RuntimeError("MODEL_IDENTITY_MISMATCH: frozen fixture contract changed")
    check_identity(read(ROOT / "source_identity.json")["files"])
    return manifest


def remaining_seconds():
    m = read(ROOT / "campaign_manifest.json")
    return m["resource_budget_s"] - (time.time() - m["resource_started_epoch"])


def require_budget():
    if remaining_seconds() <= 0:
        raise RuntimeError("BUDGET_EXHAUSTED")


def stage_identity(modules):
    paths = [Path(__file__), CONFIG, MODEL]
    paths += [PROJECT_ROOT / "v6_mujoco/postgrasp_campaign" / (m + ".py") for m in modules]
    return identity(paths)


def begin_attempt(kind, name, duration, timestep, implementation):
    require_budget()
    ledger = read(ROOT / "run_ledger.json")
    entries = ledger[kind]
    if any(e["name"] == name for e in entries):
        raise RuntimeError(f"attempt already exists; no hidden retry: {name}")
    limit = 19 if kind == "fixture_runs" else 9
    if len(entries) >= limit:
        raise RuntimeError("BUDGET_EXHAUSTED: run count")
    entries.append({"name": name, "status": "STARTED", "maximum_duration_s": duration,
        "timestep_s": timestep, "started_epoch": time.time(), "implementation_identity": implementation})
    save(ROOT / "run_ledger.json", ledger)


def end_attempt(kind, name, status, **details):
    ledger = read(ROOT / "run_ledger.json")
    item = next(e for e in ledger[kind] if e["name"] == name)
    item.update(status=status, finished_epoch=time.time(), **details)
    save(ROOT / "run_ledger.json", ledger)


def completed(path, resume, implementation):
    if not path.exists():
        return None
    if not resume:
        raise RuntimeError(f"result exists; use --resume: {path}")
    result = read(path)
    if result["implementation_identity"] != implementation:
        raise RuntimeError("MODEL_IDENTITY_MISMATCH: completed stage implementation")
    check_identity(result["input_identity"])
    for relative, expected in result["output_hashes"].items():
        if digest(ROOT / relative) != expected:
            raise RuntimeError(f"VALIDATION_FAILED: changed result {relative}")
    return result
