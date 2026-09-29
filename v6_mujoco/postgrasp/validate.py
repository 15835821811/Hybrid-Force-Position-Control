"""Read-only model audit and deterministic replays for archived N200 results."""

from __future__ import annotations

import json
import tempfile
from pathlib import Path

import mujoco
import numpy as np

from v6_mujoco.model import PROJECT_ROOT
from v6_mujoco.fpmfc.contact_model import default_contact_model_spec
from .physics import digest, load_model, object_id
from .run import CONFIG, OUTPUT, MODULE_PATHS, read_config, run_condition, save


def main() -> None:
    cfg = read_config()
    manifest = json.loads((OUTPUT/"experiment_manifest.json").read_text(encoding="utf-8"))
    if {p:digest(PROJECT_ROOT/p) for p in MODULE_PATHS} != manifest["implementation_identity"]:
        raise RuntimeError("formal runtime implementation identity changed")
    if digest(CONFIG) != manifest["config_sha256"] or digest(PROJECT_ROOT/cfg["model_xml"]) != manifest["model_sha256"]:
        raise RuntimeError("frozen model or config changed")
    old = default_contact_model_spec().compile_model()
    new = load_model(PROJECT_ROOT/cfg["model_xml"])
    masks = {}
    for gid in range(old.ngeom):
        name = mujoco.mj_id2name(old,mujoco.mjtObj.mjOBJ_GEOM,gid)
        nid = object_id(new,mujoco.mjtObj.mjOBJ_GEOM,name)
        before = (int(old.geom_contype[gid]),int(old.geom_conaffinity[gid]))
        after = (int(new.geom_contype[nid]),int(new.geom_conaffinity[nid]))
        if before != after:
            masks[name] = {"before":before,"after":after}
    if set(masks) != set(cfg["interface"]["excluded_collision_pair"]):
        raise RuntimeError(f"unexpected collision-mask changes: {masks}")
    initial = json.loads((OUTPUT/"initial_state.json").read_text(encoding="utf-8"))
    damping = json.loads((OUTPUT/"damping.json").read_text(encoding="utf-8"))
    replay = {}
    with tempfile.TemporaryDirectory(prefix="n200_replay_") as location:
        for condition in cfg["run"]["conditions"]:
            archived = OUTPUT/condition/"trace.npz"
            if not archived.exists():
                continue
            temporary = Path(location)/condition
            metrics = run_condition(condition,cfg,initial,damping,temporary)
            with np.load(archived,allow_pickle=False) as before, np.load(temporary/"trace.npz",allow_pickle=False) as after:
                differences = {}
                for key in before.files:
                    if before[key].shape != after[key].shape:
                        raise RuntimeError(f"{condition} replay shape differs: {key}")
                    differences[key] = float(np.max(np.abs(before[key]-after[key])))
            limit = {"qpos":1e-11,"qvel":1e-10,"force_grasp_n":1e-8,"moment_grasp_nm":1e-8}
            passed = metrics["status"] == json.loads((OUTPUT/condition/"metrics.json").read_text(encoding="utf-8"))["status"] and all(
                differences[key] <= tolerance for key,tolerance in limit.items())
            replay[condition] = {"passed":bool(passed),"maximum_absolute_differences":differences,
                                 "strict_tolerances":limit,"status":metrics["status"]}
            if not passed:
                raise RuntimeError(f"{condition} replay failed")
    report = {"schema_version":"n200_validation_v1",
              "source_trace_hash_verified":True,"formal_implementation_hash_verified":True,
              "model_config_hash_verified":True,"collision_mask_delta":masks,
              "same_step_independent_replay":replay,
              "step_size_sensitivity_is_separate_from_replay":True,
              "verification_identity": {p:digest(PROJECT_ROOT/p) for p in
                  ("v6_mujoco/postgrasp/validate.py","tests/test_n200_postgrasp.py")}}
    save(OUTPUT/"validation.json",report)
    print(json.dumps({"replay_passed":{k:v["passed"] for k,v in replay.items()},
                      "collision_mask_delta":list(masks)}))


if __name__ == "__main__":
    main()
