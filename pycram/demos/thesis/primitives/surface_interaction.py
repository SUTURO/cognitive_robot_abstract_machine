from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional

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
class ScrubSpec:
    radius: float
    points_per_cycle: int
    cycles: int
    z_offset: float = 0.0


@dataclass(frozen=True)
class SweepSpec:
    spacing: float
    margin: float = 0.0
    z_offset: float = 0.0


def _new_pose(frame_id: str) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    return ps


def _set_xyz(ps: PoseStamped, x: float, y: float, z: float) -> None:
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.position.z = float(z)


def bind_surface_anchor(
    surface: SurfacePlane, margin: float
) -> Optional[SurfaceAnchor]:
    hx, hy = float(surface.half_extents_xy[0]), float(surface.half_extents_xy[1])
    if hx <= margin or hy <= margin:
        return None
    p = np.array([0.0, 0.0, float(surface.z_contact)], dtype=float)
    return SurfaceAnchor(frame_id=surface.frame_id, p=p)


def compile_scrub_circle(
    anchor: SurfaceAnchor,
    surface: SurfacePlane,
    spec: ScrubSpec,
    margin: float,
) -> Iterable[PoseStamped]:
    hx, hy = float(surface.half_extents_xy[0]), float(surface.half_extents_xy[1])

    r_max = max(0.0, min(hx, hy) - float(margin))
    r = min(float(spec.radius), r_max)

    ppc = int(spec.points_per_cycle)
    if ppc <= 0:
        return []

    cycles = int(spec.cycles)
    if cycles <= 0:
        return []

    n = ppc * cycles
    out = []

    cx, cy = float(anchor.p[0]), float(anchor.p[1])
    z = float(surface.z_contact) + float(spec.z_offset)

    for i in range(n):
        a = (2.0 * math.pi) * (float(i) / float(ppc))
        x = cx + r * math.cos(a)
        y = cy + r * math.sin(a)

        if abs(x) > hx - margin or abs(y) > hy - margin:
            continue

        ps = _new_pose(anchor.frame_id)
        _set_xyz(ps, x, y, z)
        out.append(ps)

    return out


def compile_sweep_raster(
    surface: SurfacePlane,
    spec: SweepSpec,
    start_xy: Optional[np.ndarray] = None,
    end_xy: Optional[np.ndarray] = None,
) -> Iterable[SurfaceAnchor]:
    hx, hy = float(surface.half_extents_xy[0]), float(surface.half_extents_xy[1])
    m = float(spec.margin)
    s = float(spec.spacing)

    if s <= 0.0:
        return []

    if start_xy is None:
        start_xy = np.array([-(hx - m), -(hy - m)], dtype=float)
    if end_xy is None:
        end_xy = np.array([(hx - m), (hy - m)], dtype=float)

    x0, y0 = float(start_xy[0]), float(start_xy[1])
    x1, y1 = float(end_xy[0]), float(end_xy[1])

    x0 = max(-(hx - m), min((hx - m), x0))
    x1 = max(-(hx - m), min((hx - m), x1))
    y0 = max(-(hy - m), min((hy - m), y0))
    y1 = max(-(hy - m), min((hy - m), y1))

    ys = np.arange(min(y0, y1), max(y0, y1) + 1e-9, s, dtype=float)
    anchors = []

    left = min(x0, x1)
    right = max(x0, x1)

    for k, y in enumerate(ys):
        xs = [left, right] if (k % 2 == 0) else [right, left]
        for x in xs:
            p = np.array([float(x), float(y), float(surface.z_contact)], dtype=float)
            anchors.append(SurfaceAnchor(frame_id=surface.frame_id, p=p))

    return anchors


def compile_wipe_raster_scrub(
    surface: SurfacePlane,
    sweep: SweepSpec,
    scrub: ScrubSpec,
) -> Iterable[PoseStamped]:
    anchors = list(compile_sweep_raster(surface, sweep))
    out = []

    z_sweep = float(surface.z_contact) + float(sweep.z_offset)
    for a in anchors:
        ps = _new_pose(a.frame_id)
        _set_xyz(ps, float(a.p[0]), float(a.p[1]), z_sweep)
        out.append(ps)

        out.extend(compile_scrub_circle(a, surface, scrub, margin=float(sweep.margin)))

    return out
