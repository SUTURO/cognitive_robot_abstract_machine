from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from giskardpy.motion_statechart.data_types import DefaultWeights
from giskardpy.motion_statechart.goals.collision_avoidance import (
    ExternalCollisionAvoidance,
)
from giskardpy.motion_statechart.goals.wipe_goals import WipeGoal, WipeSegment
from giskardpy.motion_statechart.ros2_nodes.force_torque_monitor import (
    ForceTorqueSymbolNode,
)
from giskardpy.ros2_tools.wrench_compensation_node import COMPENSATED_WRENCH_TOPIC
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionBetweenGroups,
    AvoidExternalCollisions,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import Table
from semantic_digital_twin.spatial_types import Point3, Vector3
from semantic_digital_twin.world_description.world_entity import (
    Body,
    KinematicStructureEntity,
)

from pycram.datastructures.enums import Arms, WipeMode
from pycram.robot_plans.motions.base import BaseMotion
from pycram.view_manager import ViewManager

DEFAULT_WRENCH_TOPIC = COMPENSATED_WRENCH_TOPIC

# Fallback geometry, in m, used when no tool body is given to derive from.
_DEFAULT_LANE_SPACING = 0.05
_DEFAULT_SURFACE_MARGIN = 0.03
_DEFAULT_OBSTACLE_MARGIN = 0.06
# When a tool body *is* given, lanes overlap by this fraction of its width and
# obstacle footprints get this extra clearance, in m, beyond half the tool.
_LANE_OVERLAP = 0.3
_OBSTACLE_SAFETY = 0.02
# Default downward press, in N, expressed in the table (waypoint) frame.
_DEFAULT_PRESS_FORCE = 8.0


@dataclass
class WipeTableMotion(BaseMotion):
    """Sweep the tool over a table surface, regulating contact force with the
    admittance controller, by building a single :class:`WipeGoal`.

    ``SPILL`` wipes a continuous serpentine over the whole surface; ``CRUMB``
    runs one +x gathering stroke per lane onto a shared collection point. Where
    a stroke crosses an obstacle it is cut so the tool lifts over it. Waypoints
    are generated in the table frame; reachability is not considered here.

    Geometry that is a property of the tool (lane spacing, surface and obstacle
    margins) is derived from ``tool`` and need not be passed; obstacles default
    to the bodies the table carries (``table.objects``).
    """

    table: Table
    arm: Arms
    mode: WipeMode

    tool: Optional[Body] = None
    """Sponge/tool body. Its footprint sets the lane spacing and margins; pass
    it instead of tuning those by hand."""

    tool_contact_frame: Optional[KinematicStructureEntity] = None
    """Frame driven along the waypoints; defaults to the gripper tool frame.
    Set it to the tool's contact point (e.g. a sponge's underside)."""

    lane_spacing: Optional[float] = None
    surface_margin: Optional[float] = None
    obstacle_margin: Optional[float] = None
    """Distance between strokes / edge inset / obstacle inflation, in m. ``None``
    derives them from ``tool`` (or a default if no tool is given)."""

    stroke_sample_count: int = 2
    """Waypoints per stroke (>= 2); raise for non-flat surfaces or dense paths."""

    region: Optional[Tuple[float, float, float, float]] = None
    """``(min_x, max_x, min_y, max_y)`` sub-rectangle to wipe; ``None`` = whole top."""

    obstacles: Optional[List[Body]] = None
    """Bodies on the table to wipe around. ``None`` uses the table's own
    ``objects``; pass a list to override."""

    keep_tool_pointing_down: bool = True
    """Hold the tool's approach axis pointing straight down during the wipe."""

    approach_height: float = 0.10
    """Height above the surface for approach/retract and lifts; clear obstacles."""

    desired_force: Vector3 = field(
        default_factory=lambda: Vector3(z=_DEFAULT_PRESS_FORCE)
    )
    """Contact force the admittance balances, in the table frame (default: press
    down into the surface)."""

    contact_force_threshold: float = 3.0
    wipe_threshold: float = 0.03

    force_torque_topic: str = DEFAULT_WRENCH_TOPIC
    force_torque_reference_frame: Optional[KinematicStructureEntity] = None
    """Frame the wrench is expressed in. The compensation node keeps it in the
    sensor frame, so set this to the sensor link; it otherwise defaults to the
    tool tip, which is only correct if tool and sensor share an orientation."""

    avoid_collisions: bool = False
    """Run external collision avoidance in parallel with the wipe, letting only
    the gripper and tool touch the wiped surface. Off by default so callers that
    add their own collision rules aren't doubled up."""

    verbose: bool = False
    """Print the world-frame targets (approach, lower, waypoints, retract) when
    the motion chart is built."""

    def __post_init__(self):
        width = self._tool_contact_width()
        if self.lane_spacing is None:
            self.lane_spacing = (
                width * (1.0 - _LANE_OVERLAP) if width else _DEFAULT_LANE_SPACING
            )
        if self.surface_margin is None:
            self.surface_margin = width / 2.0 if width else _DEFAULT_SURFACE_MARGIN
        if self.obstacle_margin is None:
            self.obstacle_margin = (
                width / 2.0 + _OBSTACLE_SAFETY if width else _DEFAULT_OBSTACLE_MARGIN
            )

    def perform(self):
        return

    @property
    def _table_body(self) -> Body:
        return self.table.root

    def _tool_contact_width(self) -> Optional[float]:
        """The tool's contact-face width, in m: the smaller of its two largest
        extents (the third, smallest, is the protrusion/thickness)."""
        if self.tool is None or not self.tool.has_collision():
            return None
        box = self.tool.collision.as_bounding_box_collection_in_frame(
            self.tool
        ).bounding_box()
        extents = sorted(
            [box.max_x - box.min_x, box.max_y - box.min_y, box.max_z - box.min_z]
        )
        return extents[1]

    @property
    def _motion_chart(self) -> WipeGoal:
        tool_frame = ViewManager.get_end_effector_view(self.arm, self.robot).tool_frame
        tip_link = self.tool_contact_frame or tool_frame
        ft_node = ForceTorqueSymbolNode(
            topic_name=self.force_torque_topic,
            reference_frame=self.force_torque_reference_frame or tip_link,
            name="wipe_ft",
        )
        parallel_nodes, approach_rules, contact_rules = (
            self._collision_setup() if self.avoid_collisions else ([], [], [])
        )
        goal = WipeGoal(
            name="WipeTable",
            segments=self._build_segments(),
            tip_link=tip_link,
            root_link=self.world.root,
            ft_node=ft_node,
            contact_force_threshold=self.contact_force_threshold,
            wipe_threshold=self.wipe_threshold,
            desired_force=self.desired_force,
            approach_height=self.approach_height,
            keep_tip_axis_aligned=self.keep_tool_pointing_down,
            parallel_nodes=parallel_nodes,
            approach_collision_rules=approach_rules,
            contact_collision_rules=contact_rules,
        )
        if self.verbose:
            self._print_targets(goal)
        return goal

    def _collision_setup(self):
        """Phase-dependent collision avoidance. The whole robot (base included)
        avoids everything external for the whole wipe via a parallel
        :class:`ExternalCollisionAvoidance` constraint. The gripper and tool are
        *additionally* allowed onto the wiped surface only from the contact phase
        on, so the approach descends from above rather than driving through the
        table. Returns ``(parallel_nodes, approach_rules, contact_rules)``."""
        gripper = [b for b in self.world.bodies_with_collision if "hand_" in str(b.name)]
        tool_group = gripper + ([self.tool] if self.tool is not None else [])
        # Both phases: avoid all external collisions; the sponge is bolted to the
        # arm, so never treat it touching the arm as a collision.
        approach_rules = [AvoidExternalCollisions(robot=self.robot)]
        if self.tool is not None:
            approach_rules.append(
                AllowCollisionBetweenGroups(
                    body_group_a=[self.tool],
                    body_group_b=list(self.robot.bodies_with_collision),
                )
            )
        # Contact phase adds the gripper+tool <-> surface exemption.
        contact_rules = approach_rules + [
            AllowCollisionBetweenGroups(
                body_group_a=tool_group, body_group_b=[self._table_body]
            )
        ]
        # Weight avoidance above the wipe goals (WEIGHT_ABOVE_CA) so the QP is
        # driven away from the table rather than through it: never collide, even
        # if a far waypoint then stays out of reach.
        avoidance = ExternalCollisionAvoidance(
            robot=self.robot, weight=DefaultWeights.WEIGHT_MAX
        )
        return [avoidance], approach_rules, contact_rules

    def _print_targets(self, goal: WipeGoal) -> None:
        """Print, in the world frame, where the wipe drives ``tip_link``: the
        approach (above the surface), the contact-seeking lower target (below it,
        reached only without contact), each waypoint, and the retract -- so it is
        visible whether the tool presses onto the surface or through it."""
        tip = goal.tip_link
        tip_now = self.world.compute_forward_kinematics_np(self.world.root, tip)[:3, 3]
        print(f"tip '{tip.name}' now (world): {np.round(tip_now, 3)}")
        for seg_index, (_base_pose, waypoints) in enumerate(goal.segments):
            if not waypoints:
                continue
            surface_T = self.world.compute_forward_kinematics_np(
                self.world.root, waypoints[0].reference_frame
            )

            def to_world(x, y, z):
                return (surface_T @ np.array([float(x), float(y), float(z), 1.0]))[:3]

            first, last = waypoints[0], waypoints[-1]
            print(f"segment {seg_index} ({len(waypoints)} waypoints):")
            print(f"  approach (world): "
                  f"{np.round(to_world(first.x, first.y, float(first.z) + goal.approach_height), 3)}")
            print(f"  lower    (world): "
                  f"{np.round(to_world(first.x, first.y, float(first.z) - goal.lower_overshoot), 3)}"
                  f"  <- descends here unless force >= {goal.contact_force_threshold} N")
            for i, wp in enumerate(waypoints):
                print(f"  waypoint {i} (world): {np.round(to_world(wp.x, wp.y, wp.z), 3)}")
            print(f"  retract  (world): "
                  f"{np.round(to_world(last.x, last.y, float(last.z) + goal.approach_height), 3)}")

    def _build_segments(self) -> List[WipeSegment]:
        min_x, max_x, min_y, max_y, top_z = self._wipe_extent()
        x_start = min_x + self.surface_margin
        x_end = max_x - self.surface_margin
        lane_y_values = self._lane_offsets(
            min_y + self.surface_margin, max_y - self.surface_margin
        )
        footprints = self._obstacle_footprints()
        if self.mode == WipeMode.SPILL:
            return self._absorb_segments(x_start, x_end, lane_y_values, top_z, footprints)
        return self._collect_segments(x_start, x_end, lane_y_values, top_z, footprints)

    def _absorb_segments(self, x_start, x_end, lane_y_values, top_z, footprints):
        """One serpentine, cut into separate segments wherever it hits an obstacle."""
        waypoints: List[Point3] = []
        for lane_index, lane_y in enumerate(lane_y_values):
            stroke_start, stroke_end = (
                (x_start, x_end) if lane_index % 2 == 0 else (x_end, x_start)
            )
            waypoints.extend(self._stroke_points(stroke_start, stroke_end, lane_y, top_z))
        return [(None, run) for run in self._split_on_obstruction(waypoints, footprints)]

    def _collect_segments(self, x_start, x_end, lane_y_values, top_z, footprints):
        """One +x stroke per lane, each funneled onto a shared collection point."""
        collection_y = 0.5 * (lane_y_values[0] + lane_y_values[-1])
        collection_point = Point3(
            x=x_end, y=collection_y, z=top_z, reference_frame=self._table_body
        )
        collection_clear = not self._is_obstructed(collection_point, footprints)
        segments: List[WipeSegment] = []
        for lane_y in lane_y_values:
            runs = self._split_on_obstruction(
                self._stroke_points(x_start, x_end, lane_y, top_z), footprints
            )
            for run_index, run in enumerate(runs):
                reaches_edge = abs(float(run[-1].x) - x_end) <= 1e-9
                if (
                    run_index == len(runs) - 1
                    and reaches_edge
                    and collection_clear
                    and abs(lane_y - collection_y) > 1e-9
                ):
                    run = run + [collection_point]
                segments.append((None, run))
        return segments

    def _wipe_extent(self) -> Tuple[float, float, float, float, float]:
        min_x, max_x, min_y, max_y, top_z = self._surface_extent()
        if self.region is not None:
            region_min_x, region_max_x, region_min_y, region_max_y = self.region
            min_x = max(min_x, region_min_x)
            max_x = min(max_x, region_max_x)
            min_y = max(min_y, region_min_y)
            max_y = min(max_y, region_max_y)
        return min_x, max_x, min_y, max_y, top_z

    def _obstacle_footprints(self) -> List[Tuple[float, float, float, float]]:
        """Inflated xy footprints ``(min_x, max_x, min_y, max_y)`` per obstacle.
        Obstacles default to the bodies the table carries (``table.objects``)."""
        obstacles = (
            self.obstacles
            if self.obstacles is not None
            else [obj.root for obj in self.table.objects]
        )
        footprints: List[Tuple[float, float, float, float]] = []
        for obstacle in obstacles:
            boxes = list(
                obstacle.collision.as_bounding_box_collection_in_frame(self._table_body)
            )
            if not boxes:
                continue
            footprints.append(
                (
                    min(box.min_x for box in boxes) - self.obstacle_margin,
                    max(box.max_x for box in boxes) + self.obstacle_margin,
                    min(box.min_y for box in boxes) - self.obstacle_margin,
                    max(box.max_y for box in boxes) + self.obstacle_margin,
                )
            )
        return footprints

    def _is_obstructed(self, point: Point3, footprints) -> bool:
        x, y = float(point.x), float(point.y)
        return any(
            min_x <= x <= max_x and min_y <= y <= max_y
            for min_x, max_x, min_y, max_y in footprints
        )

    def _split_on_obstruction(self, points, footprints) -> List[List[Point3]]:
        """Split into contiguous runs, dropping points inside an obstacle footprint."""
        runs: List[List[Point3]] = []
        current: List[Point3] = []
        for point in points:
            if self._is_obstructed(point, footprints):
                if current:
                    runs.append(current)
                    current = []
            else:
                current.append(point)
        if current:
            runs.append(current)
        return runs

    def _stroke_points(self, x_start, x_end, lane_y, top_z) -> List[Point3]:
        sample_count = max(2, self.stroke_sample_count)
        return [
            Point3(
                x=x_start + i / (sample_count - 1) * (x_end - x_start),
                y=lane_y,
                z=top_z,
                reference_frame=self._table_body,
            )
            for i in range(sample_count)
        ]

    def _lane_offsets(self, start: float, end: float) -> List[float]:
        """Lane y-values from ``start`` to ``end``, spaced by at most ``lane_spacing``."""
        span = end - start
        if span <= 0.0:
            return [start]
        lane_count = max(1, math.ceil(span / self.lane_spacing))
        return [start + min(i * self.lane_spacing, span) for i in range(lane_count + 1)]

    def _surface_extent(self) -> Tuple[float, float, float, float, float]:
        """Table collision extent ``(min_x, max_x, min_y, max_y, top_z)`` in its frame."""
        body = self._table_body
        boxes = list(body.collision.as_bounding_box_collection_in_frame(body))
        if not boxes:
            raise ValueError(f"Table {body.name} has no collision geometry.")
        return (
            min(box.min_x for box in boxes),
            max(box.max_x for box in boxes),
            min(box.min_y for box in boxes),
            max(box.max_y for box in boxes),
            max(box.max_z for box in boxes),
        )
