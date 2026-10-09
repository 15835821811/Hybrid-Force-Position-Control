"""Frozen N111 input, implementation, and three-run budget contract."""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

import yaml

from ..model import PROJECT_ROOT
from .force_regulation_contract import OUTPUT_ROOT as PARENT_ROOT, load_manifest as load_parent
from .handoff_contract import sha256

OUTPUT_ROOT = PROJECT_ROOT / "output/fpmfc/n111/contact_load_compensation"
CONFIG = PROJECT_ROOT / "configs/fpmfc_n111_contact_load.yaml"
CELLS = {"gamma_0": 0.0, "gamma_05": 0.5, "gamma_10": 1.0}
RUNTIME_FILES = (
    "configs/fpmfc_n111_contact_load.yaml",
    "v6_mujoco/fpmfc/n111_contact_servo.py",
    "v6_mujoco/fpmfc/n111_execution_chain.py",
    "v6_mujoco/fpmfc/n111_contract.py",
    "v6_mujoco/fpmfc/n111_run.py",
)


def configuration() -> dict:
    value = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    if (value["schema_version"] != "n111_contact_load_v1"
            or value["baseline_commit"] != "85eec3a53a8806e87210be3843c9635f7a11da8b"
            or value["gamma_cells"] != CELLS
            or float(value["compensation_cap_fraction"]) != 0.10
            or int(value["maximum_new_closed_loop_runs"]) != 3
            or int(value["evaluation_last_steps"]) != 100):
        raise ValueError("N111 configuration differs from planned contract")
    return value


def runtime_identity() -> dict:
    return {path: sha256(PROJECT_ROOT / path) for path in RUNTIME_FILES}


def prepare_manifest() -> dict:
    cfg = configuration()
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    path = OUTPUT_ROOT / "experiment_manifest.json"
    if path.exists():
        raise FileExistsError(path)
    parent = load_parent(PARENT_ROOT / "experiment_manifest.json")
    d10 = PARENT_ROOT / "D10"
    baseline_metrics = json.loads((d10 / "metrics.json").read_text(encoding="utf-8"))
    baseline_validation = json.loads((d10 / "validation.json").read_text(encoding="utf-8"))
    source = PROJECT_ROOT / cfg["source_trace"]
    if (sha256(source) != parent["source_trace_sha256"]
            or not baseline_validation["passed"]
            or not baseline_metrics["acceptance"]["common_passed"]):
        raise RuntimeError("N110D D10 or C1 source is not qualified")
    chain = OUTPUT_ROOT / "execution_chain_summary.json"
    if not chain.exists():
        raise FileNotFoundError("finish archived D10 execution-chain reconstruction first")
    manifest = {
        "schema_version": "n111_contact_load_contract_v1",
        "frozen_at_utc": datetime.now(timezone.utc).isoformat(),
        "baseline_commit": cfg["baseline_commit"],
        "parent_manifest_sha256": sha256(PARENT_ROOT / "experiment_manifest.json"),
        "parent_runtime_identity": parent["runtime_identity"],
        "source_trace_path": cfg["source_trace"], "source_trace_sha256": sha256(source),
        "d10_trace_path": cfg["d10_trace"], "d10_trace_sha256": sha256(d10 / "trace.npz"),
        "d10_metrics_sha256": sha256(d10 / "metrics.json"),
        "d10_validation_sha256": sha256(d10 / "validation.json"),
        "contact_config_path": cfg["contact_config"],
        "contact_config_sha256": sha256(PROJECT_ROOT / cfg["contact_config"]),
        "execution_chain_sha256": sha256(chain),
        "gamma_cells": CELLS, "compensation_cap_fraction": 0.10,
        "maximum_new_closed_loop_runs": 3, "run_order": list(CELLS),
        "gamma_zero_state_atol": float(cfg["gamma_zero_state_atol"]),
        "gamma_zero_torque_atol_nm": float(cfg["gamma_zero_torque_atol_nm"]),
        "evaluation_last_steps": 100, "force_rmse_gate_n": 0.3,
        "runtime_identity": runtime_identity(),
        "protocol": "C1 source, S synchronized held-torque feedback and homogeneous HQP, D10 M1 B100 K0 reset, 2 ms RK4, 20 ms HQP, one second; gamma=0 then 0.5 if compatible then 1 if 0.5 common-safe",
    }
    path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    (OUTPUT_ROOT / "run_ledger.json").write_text(json.dumps({"attempts": [], "budget": 3}, indent=2)+"\n", encoding="utf-8")
    return manifest


def load_manifest() -> dict:
    path = OUTPUT_ROOT / "experiment_manifest.json"
    value = json.loads(path.read_text(encoding="utf-8"))
    cfg = configuration()
    if value["schema_version"] != "n111_contact_load_contract_v1":
        raise ValueError("N111 manifest schema differs")
    if runtime_identity() != value["runtime_identity"]:
        raise RuntimeError("N111 runtime source changed after freezing")
    for key, file in (("source_trace_sha256", cfg["source_trace"]),
                      ("d10_trace_sha256", cfg["d10_trace"]),
                      ("contact_config_sha256", cfg["contact_config"])):
        if sha256(PROJECT_ROOT / file) != value[key]:
            raise RuntimeError(f"frozen input changed: {key}")
    if sha256(PARENT_ROOT / "experiment_manifest.json") != value["parent_manifest_sha256"]:
        raise RuntimeError("parent manifest changed")
    return value
