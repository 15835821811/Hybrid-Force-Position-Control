"""Audited MuJoCo model contract reconstructed from the source Simscape model."""

from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import mujoco
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
MODEL_XML = PROJECT_ROOT / "models" / "flexiv_rizon4s_scene.xml"
MESH_ROOT = PROJECT_ROOT / "assets" / "meshes"
SOURCE_URDF = PROJECT_ROOT / "assets" / "new_of_flexiv_rizon4s_kinematics.urdf"

CONTRACT_VERSION = "flexiv_rizon4s_simscape_to_mujoco_v1"
JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 8))
ACTUATOR_NAMES = tuple(f"torque_joint{index}" for index in range(1, 8))
COLLISION_GEOM_NAMES = tuple(f"link{index}_collision" for index in range(8))
HOME_JOINT_POSITION = np.deg2rad(np.asarray([0.0, -40.0, 0.0, 90.0, 0.0, 40.0, 0.0]))
SIMSCAPE_HOME_FLANGE_POSITION_M = np.asarray(
    [0.687202096629, -0.109788289891, 0.493502009119], dtype=np.float64
)


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _object_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    value = mujoco.mj_name2id(model, kind, name)
    if value < 0:
        raise ValueError(f"MuJoCo object {name!r} of type {kind} is missing")
    return int(value)


@dataclass(frozen=True)
class FlexivModelSpec:
    """Physical and controller parameters fixed by the migration contract."""

    model_xml: Path = MODEL_XML
    mesh_root: Path = MESH_ROOT
    source_urdf: Path = SOURCE_URDF
    timestep_s: float = 0.002
    task_period_s: float = 0.020
    integrator: str = "RK4"
    joint_names: tuple[str, ...] = JOINT_NAMES
    actuator_names: tuple[str, ...] = ACTUATOR_NAMES
    home_joint_position: np.ndarray = HOME_JOINT_POSITION
    torque_limits_nm: np.ndarray = np.asarray(
        [123.0, 123.0, 64.0, 64.0, 39.0, 39.0, 39.0], dtype=np.float64
    )
    velocity_limits_rad_s: np.ndarray = np.asarray(
        [2.0944, 2.0944, 2.4435, 2.4435, 4.8869, 4.8869, 4.8869],
        dtype=np.float64,
    )
    acceleration_limits_rad_s2: np.ndarray = np.full(7, 4.0, dtype=np.float64)
    position_kp: np.ndarray = np.full(7, 200.0, dtype=np.float64)
    velocity_kd: np.ndarray = np.full(7, 20.0, dtype=np.float64)

    def validate(self) -> None:
        for path in (self.model_xml, self.source_urdf):
            if not Path(path).is_file():
                raise FileNotFoundError(path)
        if len(self.joint_names) != 7 or len(self.actuator_names) != 7:
            raise ValueError("the migrated plant must expose seven joints and actuators")
        arrays = (
            self.home_joint_position,
            self.torque_limits_nm,
            self.velocity_limits_rad_s,
            self.acceleration_limits_rad_s2,
            self.position_kp,
            self.velocity_kd,
        )
        if any(np.asarray(value).shape != (7,) for value in arrays):
            raise ValueError("all per-joint model arrays must have shape (7,)")
        if min(float(self.timestep_s), float(self.task_period_s)) <= 0.0:
            raise ValueError("simulation periods must be positive")
        if abs(self.task_period_s / self.timestep_s - 10.0) > 1e-12:
            raise ValueError("one 50 Hz task tick must contain ten 500 Hz steps")
        if self.integrator != "RK4":
            raise ValueError("the orbital dynamics contract requires the RK4 integrator")

    def _xml_and_assets(self) -> tuple[str, dict[str, bytes]]:
        """Load through bytes so a Chinese workspace path is supported by MuJoCo."""

        xml = Path(self.model_xml).read_text(encoding="utf-8")
        xml = xml.replace('meshdir="../assets/meshes"', 'meshdir="."')
        assets: dict[str, bytes] = {}
        for index in range(8):
            relative = Path("rizon4s") / "collision" / f"link{index}.stl"
            path = Path(self.mesh_root) / relative
            if not path.is_file():
                raise FileNotFoundError(path)
            assets[relative.as_posix()] = path.read_bytes()
        return xml, assets

    def compile_model(self) -> mujoco.MjModel:
        self.validate()
        xml, assets = self._xml_and_assets()
        model = mujoco.MjModel.from_xml_string(xml, assets)
        if (model.nq, model.nv, model.nu) != (14, 13, 7):
            raise RuntimeError(
                f"unexpected state dimensions {(model.nq, model.nv, model.nu)}"
            )
        if abs(float(model.opt.timestep) - self.timestep_s) > 1e-12:
            raise RuntimeError("compiled MuJoCo timestep differs from the contract")
        if np.linalg.norm(model.opt.gravity) > 1e-12:
            raise RuntimeError("the migrated orbital plant must use zero gravity")
        if int(model.opt.integrator) != int(mujoco.mjtIntegrator.mjINT_RK4):
            raise RuntimeError("compiled MuJoCo integrator differs from the RK4 contract")
        for name in self.joint_names:
            _object_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        for name in self.actuator_names:
            _object_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        _object_id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_free_joint")
        _object_id(model, mujoco.mjtObj.mjOBJ_SITE, "flange_site")
        _object_id(model, mujoco.mjtObj.mjOBJ_BODY, "tumbling_target")
        _object_id(model, mujoco.mjtObj.mjOBJ_GEOM, "tumbling_target_geom")
        return model

    def joint_addresses(self, model: mujoco.MjModel) -> tuple[np.ndarray, np.ndarray]:
        qpos = []
        dofs = []
        for name in self.joint_names:
            joint_id = _object_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            qpos.append(int(model.jnt_qposadr[joint_id]))
            dofs.append(int(model.jnt_dofadr[joint_id]))
        return np.asarray(qpos, dtype=np.int32), np.asarray(dofs, dtype=np.int32)

    @staticmethod
    def base_slices(model: mujoco.MjModel) -> tuple[slice, slice]:
        joint_id = _object_id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_free_joint")
        return (
            slice(int(model.jnt_qposadr[joint_id]), int(model.jnt_qposadr[joint_id]) + 7),
            slice(int(model.jnt_dofadr[joint_id]), int(model.jnt_dofadr[joint_id]) + 6),
        )

    def reset_home(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        key_id = _object_id(model, mujoco.mjtObj.mjOBJ_KEY, "simscape_home")
        mujoco.mj_resetDataKeyframe(model, data, key_id)
        data.qvel[:] = 0.0
        data.ctrl[:] = 0.0
        mujoco.mj_forward(model, data)

    def source_bundle_sha256(self) -> str:
        digest = hashlib.sha256()
        paths = [Path(self.model_xml), Path(self.source_urdf)] + [
            Path(self.mesh_root) / "rizon4s" / "collision" / f"link{index}.stl"
            for index in range(8)
        ]
        for path in paths:
            digest.update(path.name.encode("utf-8"))
            digest.update(b"\0")
            digest.update(bytes.fromhex(_sha256_file(path)))
        return digest.hexdigest()

    def identity(self) -> dict[str, Any]:
        payload = {
            "contract_version": CONTRACT_VERSION,
            "source_bundle_sha256": self.source_bundle_sha256(),
            "mujoco_version": mujoco.__version__,
            "joint_names": list(self.joint_names),
            "actuator_names": list(self.actuator_names),
            "home_joint_position_rad": self.home_joint_position.tolist(),
            "torque_limits_nm": self.torque_limits_nm.tolist(),
            "timestep_s": self.timestep_s,
            "task_period_s": self.task_period_s,
            "integrator": self.integrator,
            "base_mass_kg": 500.0,
            "base_diagonal_inertia_kg_m2": [20.833333333] * 3,
            "actuation": "seven_direct_joint_torque_motors",
        }
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
        payload["runtime_contract_sha256"] = hashlib.sha256(encoded).hexdigest()
        return payload


def default_model_spec() -> FlexivModelSpec:
    return FlexivModelSpec()


def body_id(model: mujoco.MjModel, name: str) -> int:
    return _object_id(model, mujoco.mjtObj.mjOBJ_BODY, name)


def geom_id(model: mujoco.MjModel, name: str) -> int:
    return _object_id(model, mujoco.mjtObj.mjOBJ_GEOM, name)


def site_id(model: mujoco.MjModel, name: str) -> int:
    return _object_id(model, mujoco.mjtObj.mjOBJ_SITE, name)
