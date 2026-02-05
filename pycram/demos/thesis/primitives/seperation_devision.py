from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import Iterable, Optional, List, Protocol

import numpy as np
from geometry_msgs.msg import PoseStamped


@dataclass(frozen=True)
class CutPlane:
    frame_id: str
    half_extents: np.ndarray


@dataclass(frozen=True)
class CutAnchor:
    frame_id: str
    p: np.ndarray


@dataclass(frozen=True)
class SliceSpec:
    slice_thickness: float
    z_clearance: float
    z_cut: float
    margin_xy: float = 0.0


def _new_pose(frame_id: str) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    return ps


def _set_xyz(ps: PoseStamped, x: float, y: float, z: float) -> None:
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.position.z = float(z)


def bind_slice_anchors_along_x(
    obj_frame_id: str,
    half_extents: np.ndarray,
    spec: SliceSpec,
) -> List[CutAnchor]:
    hx, hy, hz = float(half_extents[0]), float(half_extents[1]), float(half_extents[2])

    t = float(spec.slice_thickness)
    if t <= 0.0:
        return []

    x_min = -(hx - float(spec.margin_xy))
    x_max = +(hx - float(spec.margin_xy))
    if x_max <= x_min:
        return []

    n = int((x_max - x_min) / t)
    if n <= 0:
        return []

    start = x_min + 0.5 * t
    anchors = []
    for i in range(n):
        x = start + float(i) * t
        anchors.append(
            CutAnchor(frame_id=obj_frame_id, p=np.array([x, 0.0, 0.0], dtype=float))
        )

    return anchors


def compile_slice_down_through_up(
    anchor: CutAnchor,
    obj_half_extents: np.ndarray,
    spec: SliceSpec,
) -> Iterable[PoseStamped]:
    hz = float(obj_half_extents[2])

    z_top = hz + float(spec.z_clearance)
    z_cut = float(spec.z_cut)

    x = float(anchor.p[0])
    y = float(anchor.p[1])

    out = []

    p0 = _new_pose(anchor.frame_id)
    _set_xyz(p0, x, y, z_top)
    out.append(p0)

    p1 = _new_pose(anchor.frame_id)
    _set_xyz(p1, x, y, z_cut)
    out.append(p1)

    p2 = _new_pose(anchor.frame_id)
    _set_xyz(p2, x, y, z_top)
    out.append(p2)

    return out


class QuaternionOps(Protocol):
    def axis_angle(self, axis: np.ndarray, angle_deg: float) -> object: ...


@dataclass(frozen=True)
class CutAnchor:
    frame_id: str
    p: np.ndarray


class CutSide(Enum):
    POS_Y = auto()
    NEG_Y = auto()


@dataclass(frozen=True)
class SawSliceSpec:
    slice_thickness: float
    tool_half_length: float

    prelift_z: float
    insert_z: float

    y_standoff: float

    tilt_y_deg: float
    pitch_x_deg: float

    stroke_x: float

    rotate_y_shift: float
    rotate_x_shift: float

    return_y_shift: float
    return_x_shift: float

    final_lift_z: float


def _rot(ps: PoseStamped, q: object) -> None:
    ps.pose.orientation.x = float(q[0])
    ps.pose.orientation.y = float(q[1])
    ps.pose.orientation.z = float(q[2])
    ps.pose.orientation.w = float(q[3])


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ax, ay, az, aw = float(a[0]), float(a[1]), float(a[2]), float(a[3])
    bx, by, bz, bw = float(b[0]), float(b[1]), float(b[2]), float(b[3])
    return np.array(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ],
        dtype=float,
    )


def axis_angle_quat(axis: np.ndarray, angle_deg: float) -> np.ndarray:
    a = np.asarray(axis, dtype=float)
    a = a / max(np.linalg.norm(a), 1e-12)
    th = np.deg2rad(float(angle_deg))
    s = np.sin(0.5 * th)
    return np.array([a[0] * s, a[1] * s, a[2] * s, np.cos(0.5 * th)], dtype=float)


def bind_cut_side_from_robot_y(angle_y: float) -> CutSide:
    return CutSide.NEG_Y if float(angle_y) >= 0.0 else CutSide.POS_Y


def compile_saw_slice_sequence(
    anchor: CutAnchor,
    side: CutSide,
    spec: SawSliceSpec,
    base_orientation: Optional[np.ndarray] = None,
) -> Iterable[PoseStamped]:
    sgn = +1.0 if side is CutSide.POS_Y else -1.0

    x0 = float(anchor.p[0])
    y0 = float(anchor.p[1]) + sgn * (
        float(spec.tool_half_length) + float(spec.y_standoff)
    )
    z_lift = float(spec.prelift_z)
    z_ins = float(spec.insert_z)

    q = (
        np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        if base_orientation is None
        else np.asarray(base_orientation, dtype=float)
    )

    q_tilt = axis_angle_quat(
        np.array([0.0, 1.0, 0.0], dtype=float), float(spec.tilt_y_deg)
    )
    q = _quat_mul(q, q_tilt)

    ps0 = _new_pose(anchor.frame_id)
    _set_xyz(ps0, x0, y0, z_lift)
    _rot(ps0, q)

    ps1 = _new_pose(anchor.frame_id)
    _set_xyz(ps1, x0, y0, z_ins)
    _rot(ps1, q)

    ps2 = _new_pose(anchor.frame_id)
    _set_xyz(ps2, x0 + float(spec.stroke_x), y0, z_ins)
    _rot(ps2, q)

    q_pitch = axis_angle_quat(
        np.array([1.0, 0.0, 0.0], dtype=float), float(spec.pitch_x_deg)
    )
    q2 = _quat_mul(q, q_pitch)

    ps3 = _new_pose(anchor.frame_id)
    _set_xyz(
        ps3, x0 + float(spec.stroke_x), y0 - sgn * float(spec.rotate_y_shift), z_ins
    )
    _rot(ps3, q2)

    ps4 = _new_pose(anchor.frame_id)
    _set_xyz(ps4, x0, y0 - sgn * float(spec.return_y_shift), z_ins)
    _rot(ps4, q2)

    ps5 = _new_pose(anchor.frame_id)
    _set_xyz(
        ps5,
        x0 - float(spec.return_x_shift),
        y0 - sgn * float(spec.return_y_shift),
        float(spec.final_lift_z),
    )
    _rot(ps5, q2)

    ps6 = _new_pose(anchor.frame_id)
    _set_xyz(ps6, x0, y0, z_lift)
    _rot(ps6, q)

    return [ps0, ps1, ps2, ps3, ps4, ps5, ps6]
