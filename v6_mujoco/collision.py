"""Audited MuJoCo signed-distance pairs for the migrated single arm."""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from .model import geom_id


@dataclass(frozen=True)
class CollisionPair:
    name: str
    geom_a: int
    geom_b: int
    category: str


def build_collision_pairs(model: mujoco.MjModel) -> tuple[CollisionPair, ...]:
    links = [geom_id(model, f"link{i}_collision") for i in range(8)]
    obstacle = geom_id(model, "workspace_obstacle_0")
    satellite = geom_id(model, "satellite_base_collision")
    pairs: list[CollisionPair] = []
    for index, link in enumerate(links):
        pairs.append(CollisionPair(f"link{index}_to_obstacle", link, obstacle, "workspace"))
    for index in range(2, 8):
        pairs.append(CollisionPair(f"link{index}_to_satellite", links[index], satellite, "satellite"))
    # Match the reference verifier's ancestor-exclusion depth of three.
    for left in range(8):
        for right in range(left + 4, 8):
            pairs.append(CollisionPair(f"link{left}_to_link{right}", links[left], links[right], "self"))
    return tuple(pairs)


def signed_distance(model: mujoco.MjModel, data: mujoco.MjData, pair: CollisionPair, maximum: float = 2.0) -> float:
    return float(mujoco.mj_geomDistance(model, data, pair.geom_a, pair.geom_b, maximum, np.zeros(6)))


def minimum_signed_distance(model: mujoco.MjModel, data: mujoco.MjData, pairs: tuple[CollisionPair, ...]) -> tuple[float, str]:
    values = [(signed_distance(model, data, pair), pair.name) for pair in pairs]
    return min(values, key=lambda item: item[0])
