from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Protocol, Tuple, Optional, Iterable

import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped

from pycram.demos.thesis.geometry.volume_models import ContainerVolumeModel
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.geometry import Cylinder, Box, Scale


@dataclass(frozen=True)
class AgitationSpec:
    turns: int
    angle_step_deg: float
    radius_step: float
    z_step: float
    start_radius: float = 0.0
    start_angle_deg: float = 0.0


@dataclass(frozen=True)
class VolumeAnchor:
    frame_id: str
    p: np.ndarray


def _new_pose(frame_id: str) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    return ps


def _set_xyz(ps: PoseStamped, x: float, y: float, z: float) -> None:
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.position.z = float(z)


def compile_volume_agitation(
    anchor: VolumeAnchor,
    spec: AgitationSpec,
    volume: Optional[ContainerVolumeModel] = None,
    epsilon: float = 0.0,
) -> Iterable[PoseStamped]:
    angle_step = math.radians(float(spec.angle_step_deg))
    start_angle = math.radians(float(spec.start_angle_deg))

    if angle_step <= 0.0:
        return []

    steps_per_turn = int(round((2.0 * math.pi) / angle_step))
    if steps_per_turn <= 0:
        return []

    n = int(spec.turns) * steps_per_turn
    if n <= 0:
        return []

    cx, cy, cz = float(anchor.p[0]), float(anchor.p[1]), float(anchor.p[2])

    r_max = None
    if volume is not None:
        c0 = float(volume.distance_inside(np.array([cx, cy, cz], dtype=float)))
        r_max = max(0.0, c0 - float(epsilon))

    out = []
    for i in range(n):
        r = float(spec.start_radius) + float(spec.radius_step) * float(i)
        if r_max is not None:
            r = min(r, r_max)

        a = start_angle + angle_step * float(i)
        z = cz + float(spec.z_step) * float(i)

        x = cx + r * math.cos(a)
        y = cy + r * math.sin(a)

        if volume is not None:
            d = float(volume.distance_inside(np.array([x, y, z], dtype=float)))
            if d < float(epsilon):
                continue

        ps = _new_pose(anchor.frame_id)
        _set_xyz(ps, x, y, z)
        out.append(ps)

    return out
