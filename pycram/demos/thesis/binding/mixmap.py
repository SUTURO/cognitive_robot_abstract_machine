from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Protocol, Tuple, Optional, Iterable

import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped

from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.geometry import Cylinder, Box, Scale


class MixScore(Enum):
    MAX_CLEARANCE = auto()


@dataclass(frozen=True)
class MixmapParams:
    z_ratio: float
    epsilon: float
    grid_step: float
    score: MixScore = MixScore.MAX_CLEARANCE


@dataclass(frozen=True)
class MixmapResult:
    p_in_container: np.ndarray
    z_mix: float
    clearance: float


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def solve_mixmap(
    volume: ContainerVolumeModel, params: MixmapParams
) -> Optional[MixmapResult]:
    z_ratio = _clamp(float(params.z_ratio), 0.0, 1.0)
    h = float(volume.inner_height())
    z_mix = (-0.5 * h) + z_ratio * h

    eps = float(params.epsilon)

    p0 = np.array([0.0, 0.0, float(z_mix)], dtype=float)
    d0 = float(volume.distance_inside(p0))
    if d0 >= eps:
        return MixmapResult(p_in_container=p0, z_mix=float(z_mix), clearance=float(d0))

    hx, hy = volume.inner_bounds_xy()
    step = float(params.grid_step)
    if step <= 0.0:
        return None

    xs = np.arange(-hx, hx + 1e-9, step, dtype=float)
    ys = np.arange(-hy, hy + 1e-9, step, dtype=float)

    best_p: Optional[np.ndarray] = None
    best_clear = -1e30
    best_center_cost = 1e30
    tie_eps = 1e-12

    for x in xs:
        for y in ys:
            p = np.array([float(x), float(y), float(z_mix)], dtype=float)
            d = float(volume.distance_inside(p))
            if d < eps:
                continue

            center_cost = float(x) * float(x) + float(y) * float(y)

            if d > best_clear + tie_eps:
                best_clear = d
                best_p = p
                best_center_cost = center_cost
            elif abs(d - best_clear) <= tie_eps and center_cost < best_center_cost:
                best_p = p
                best_center_cost = center_cost

    if best_p is None:
        return None

    return MixmapResult(
        p_in_container=best_p, z_mix=float(z_mix), clearance=float(best_clear)
    )
