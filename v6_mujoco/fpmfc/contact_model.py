"""Independent MuJoCo contract for the physical-target contact extension."""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from ..model import (
    MESH_ROOT,
    SOURCE_URDF,
    FlexivModelSpec,
    PROJECT_ROOT,
    body_id,
    geom_id,
    site_id,
)
from .contact_config import DEFAULT_CONTACT_CONFIG_PATH, load_contact_config


CONTACT_MODEL_XML = PROJECT_ROOT / "models" / "flexiv_rizon4s_contact_scene.xml"
CONTACT_CONTRACT_VERSION = "flexiv_rizon4s_physical_contact_v1"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


@dataclass(frozen=True)
class ContactModelSpec(FlexivModelSpec):
    model_xml: Path = CONTACT_MODEL_XML
    mesh_root: Path = MESH_ROOT
    source_urdf: Path = SOURCE_URDF
    contact_config_path: Path = DEFAULT_CONTACT_CONFIG_PATH

    def compile_model(self) -> mujoco.MjModel:
        self.validate()
        contact_config = load_contact_config(self.contact_config_path)
        xml, assets = self._xml_and_assets()
        model = mujoco.MjModel.from_xml_string(xml, assets)
        if (model.nq, model.nv, model.nu) != (21, 19, 7):
            raise RuntimeError(
                f"unexpected contact state dimensions {(model.nq, model.nv, model.nu)}"
            )
        if abs(float(model.opt.timestep) - self.timestep_s) > 1e-12:
            raise RuntimeError("compiled contact timestep differs from the contract")
        if np.linalg.norm(model.opt.gravity) > 1e-12:
            raise RuntimeError("the contact scenario must remain zero gravity")
        if int(model.opt.integrator) != int(mujoco.mjtIntegrator.mjINT_RK4):
            raise RuntimeError("the contact scenario must use RK4")
        for name in self.joint_names:
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        for name in self.actuator_names:
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        for name in ("base_free_joint", "target_free_joint"):
            if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) < 0:
                raise ValueError(f"contact joint {name!r} is missing")
        for name in ("flange_site", "target_center_frame", "target_grasp_site"):
            site_id(model, name)
        for name in (
            "gripper_contact_pad",
            "tumbling_target_geom",
            "target_contact_plate",
        ):
            geom_id(model, name)

        target_body = body_id(model, "tumbling_target")
        target_values = contact_config["physical_target"]
        if not np.isclose(
            model.body_mass[target_body], float(target_values["mass_kg"]), atol=1e-12
        ):
            raise RuntimeError("contact XML target mass differs from contact config")
        if not np.allclose(
            model.body_inertia[target_body],
            np.asarray(target_values["diagonal_inertia_kg_m2"], dtype=np.float64),
            atol=1e-12,
        ):
            raise RuntimeError("contact XML target inertia differs from contact config")
        pad = geom_id(model, "gripper_contact_pad")
        target_geom = geom_id(model, "target_contact_plate")
        if not (
            int(model.geom_contype[pad]) == 2
            and int(model.geom_conaffinity[pad]) == 4
            and int(model.geom_contype[target_geom]) == 4
            and int(model.geom_conaffinity[target_geom]) == 2
        ):
            raise RuntimeError("contact interface collision masks differ from the contract")
        expected_margin = float(contact_config["interface"]["contact_margin_m"])
        if not np.allclose(
            model.geom_margin[[pad, target_geom]], expected_margin, atol=1e-12
        ):
            raise RuntimeError("contact interface margin differs from the contract")
        interface = contact_config["interface"]
        expected_pad_size = np.asarray(
            [
                float(interface["tool_pad_radius_m"]),
                float(interface["tool_pad_half_thickness_m"]),
                0.0,
            ]
        )
        if not np.allclose(model.geom_size[pad], expected_pad_size, atol=1e-12):
            raise RuntimeError("contact pad dimensions differ from the contract")
        expected_pad_z = -(
            float(interface["tool_pad_half_thickness_m"])
            + float(interface["tool_pad_face_recess_m"])
        )
        if not np.isclose(model.geom_pos[pad, 2], expected_pad_z, atol=1e-12):
            raise RuntimeError("contact pad face recess differs from the contract")
        if not np.allclose(
            model.geom_size[target_geom],
            np.asarray(interface["target_plate_half_size_m"]),
            atol=1e-12,
        ):
            raise RuntimeError("target interface dimensions differ from the contract")
        target_grasp = site_id(model, "target_grasp_site")
        target_plate_outer_y = (
            float(model.geom_pos[target_geom, 1])
            - float(model.geom_size[target_geom, 2])
        )
        if not np.isclose(
            target_plate_outer_y,
            float(model.site_pos[target_grasp, 1]),
            atol=1e-12,
        ):
            raise RuntimeError("target contact plate is not flush with the grasp site")
        expected_solref = np.asarray(
            [
                float(interface["solver_time_constant_s"]),
                float(interface["solver_damping_ratio"]),
            ]
        )
        if not np.allclose(
            model.geom_solref[[pad, target_geom]], expected_solref, atol=1e-12
        ):
            raise RuntimeError("contact solver reference differs from the contract")
        return model

    @staticmethod
    def target_slices(model: mujoco.MjModel) -> tuple[slice, slice]:
        joint = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, "target_free_joint"
        )
        if joint < 0:
            raise ValueError("target_free_joint is missing")
        return (
            slice(int(model.jnt_qposadr[joint]), int(model.jnt_qposadr[joint]) + 7),
            slice(int(model.jnt_dofadr[joint]), int(model.jnt_dofadr[joint]) + 6),
        )

    def identity(self) -> dict[str, Any]:
        payload = {
            "contract_version": CONTACT_CONTRACT_VERSION,
            "model_xml_sha256": _sha256(Path(self.model_xml)),
            "source_bundle_sha256": self.source_bundle_sha256(),
            "contact_config_sha256": _sha256(Path(self.contact_config_path)),
            "mujoco_version": mujoco.__version__,
            "state_dimensions": [21, 19, 7],
            "timestep_s": self.timestep_s,
            "task_period_s": self.task_period_s,
            "integrator": self.integrator,
            "interface_geoms": ["gripper_contact_pad", "target_contact_plate"],
        }
        encoded = json.dumps(
            payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False
        ).encode("utf-8")
        payload["runtime_contract_sha256"] = hashlib.sha256(encoded).hexdigest()
        return payload


def default_contact_model_spec() -> ContactModelSpec:
    return ContactModelSpec()
