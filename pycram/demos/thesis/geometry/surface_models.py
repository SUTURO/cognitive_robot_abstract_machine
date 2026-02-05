from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Iterable

import numpy as np
from geometry_msgs.msg import PoseStamped


@dataclass(frozen=True)
class SurfacePlane:
    frame_id: str
    half_extents_xy: np.ndarray
    z_contact: float = 0.0


@dataclass(frozen=True)
class SurfaceAnchor:
    frame_id: str
    p: np.ndarray


@dataclass(frozen=True)
class WipeSpec:
    pattern_points: int
    radius: float
    cycles: int
    z_offset: float = 0.0


def bind_surface_anchor(
    surface: SurfacePlane,
    margin: float,
) -> Optional[SurfaceAnchor]:
    hx, hy = float(surface.half_extents_xy[0]), float(surface.half_extents_xy[1])

    if hx <= margin or hy <= margin:
        return None

    p = np.array([0.0, 0.0, float(surface.z_contact)], dtype=float)
    return SurfaceAnchor(frame_id=surface.frame_id, p=p)


def _new_pose(frame_id: str) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    return ps


def _set_xyz(ps: PoseStamped, x: float, y: float, z: float) -> None:
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.position.z = float(z)


def compile_circular_wipe(
    anchor: SurfaceAnchor,
    surface: SurfacePlane,
    spec: WipeSpec,
    margin: float,
) -> Iterable[PoseStamped]:
    hx, hy = float(surface.half_extents_xy[0]), float(surface.half_extents_xy[1])

    r_max = max(0.0, min(hx, hy) - float(margin))
    r = min(float(spec.radius), r_max)

    n = int(spec.pattern_points) * int(spec.cycles)
    if n <= 0:
        return []

    out = []
    for i in range(n):
        a = (2.0 * math.pi) * (float(i) / float(spec.pattern_points))
        x = float(anchor.p[0]) + r * math.cos(a)
        y = float(anchor.p[1]) + r * math.sin(a)
        z = float(surface.z_contact) + float(spec.z_offset)

        if abs(x) > hx - margin or abs(y) > hy - margin:
            continue

        ps = _new_pose(anchor.frame_id)
        _set_xyz(ps, x, y, z)
        out.append(ps)

    return out
