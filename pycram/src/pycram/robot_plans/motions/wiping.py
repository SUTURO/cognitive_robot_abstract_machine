from __future__ import annotations

import math
from collections.abc import Callable
from copy import deepcopy
from dataclasses import dataclass, field

import numpy as np

from giskardpy.executor import Executor
from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.data_types import DefaultWeights
from giskardpy.motion_statechart.goals.collision_avoidance import (
    ExternalCollisionAvoidance,
)
from giskardpy.motion_statechart.goals.wipe_goals import (
    AlignTipAxis,
    WipeGoal,
    WipeSegment,
)
from giskardpy.motion_statechart.graph_node import EndMotion, MotionStatechartNode
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy.motion_statechart.ros2_nodes.force_torque_monitor import (
    ForceTorqueSymbolNode,
)
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPosition
from giskardpy.qp.qp_controller_config import QPControllerConfig
from giskardpy.ros2_tools.wrench_compensation_node import COMPENSATED_WRENCH_TOPIC
from krrood.symbolic_math.symbolic_math import trinary_logic_and
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionBetweenGroups,
    AvoidExternalCollisions,
    CollisionRule,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import Table
from semantic_digital_twin.spatial_types import Point3, Vector3
from semantic_digital_twin.world_description.world_entity import (
    Body,
    KinematicStructureEntity,
)

from pycram.datastructures.enums import Arms, WipeMode
from pycram.exceptions import UncontrolledRobotBaseError
from pycram.robot_plans.motions.base import BaseMotion
from pycram.view_manager import ViewManager

Rectangle = tuple[float, float, float, float]
"""Axis-aligned ``(min_x, max_x, min_y, max_y)`` bounds in the table frame."""

# Geometry is derived from the tool's contact width (a default when none is
# given): lanes overlap by this fraction of it, obstacle footprints are
# inflated by half of it plus this clearance, in m.
_DEFAULT_TOOL_WIDTH = 0.07
_LANE_OVERLAP = 0.3
_OBSTACLE_SAFETY = 0.02
# Depth resolution of the reach measurement's bisection, in m.
_REACH_PROBE_TOLERANCE = 0.05


def _segment_crosses_rectangle(start: Point3, end: Point3, rectangle: Rectangle) -> bool:
    """Whether the xy segment ``start``-``end`` intersects the axis-aligned
    ``rectangle``, via Liang-Barsky clipping."""
    min_x, max_x, min_y, max_y = rectangle
    start_x, start_y = float(start.x), float(start.y)
    delta_x = float(end.x) - start_x
    delta_y = float(end.y) - start_y
    enter_fraction, exit_fraction = 0.0, 1.0
    for delta, low, high in (
        (delta_x, min_x - start_x, max_x - start_x),
        (delta_y, min_y - start_y, max_y - start_y),
    ):
        if abs(delta) < 1e-12:
            if low > 0.0 or high < 0.0:
                return False
            continue
        fraction_low, fraction_high = sorted((low / delta, high / delta))
        enter_fraction = max(enter_fraction, fraction_low)
        exit_fraction = min(exit_fraction, fraction_high)
        if enter_fraction > exit_fraction:
            return False
    return True


@dataclass(frozen=True)
class ReachProbeResult:
    """Outcome of one reach probe."""

    converged: bool
    """Whether the tip settled on the probed point with the tool pointing down."""

    base_x: float
    """Base position along the table's x-axis at the end of the probe, in m."""

    base_y: float
    """Base position along the table's y-axis at the end of the probe, in m."""


@dataclass(frozen=True)
class ReachLimit:
    """The measured reachable band of the wipe area in the table's xy-plane: a
    trapezoid between the shallow row (nearest the robot) and the deepest
    reachable row, since the band narrows with depth."""

    base_x: float
    """Base position along the table's x-axis when measured, in m."""

    base_y: float
    """Base position along the table's y-axis when measured, in m."""

    direction_x: float
    """X-component of the unit approach direction (axis-aligned in the table
    frame), base toward the wipe area."""

    direction_y: float
    """Y-component of the unit approach direction."""

    shallow_depth: float
    """Depth of the wipe area's nearest row, where the band starts, in m."""

    depth: float
    """Largest reachable offset from the base along the approach direction, in m."""

    shallow_lateral_min: float
    """Smallest reachable lateral offset at the shallow row, in m."""

    shallow_lateral_max: float
    """Largest reachable lateral offset at the shallow row, in m."""

    deep_lateral_min: float
    """Smallest reachable lateral offset at the deepest row, in m."""

    deep_lateral_max: float
    """Largest reachable lateral offset at the deepest row, in m."""

    def contains(self, point: Point3) -> bool:
        """Whether the table-frame ``point`` lies within the measured reach."""
        point_depth = self.depth_of(point)
        if point_depth > self.depth:
            return False
        lateral_min, lateral_max = self.lateral_bounds_at(point_depth)
        return lateral_min <= self.lateral_of(point) <= lateral_max

    def lateral_bounds_at(self, depth: float) -> tuple[float, float]:
        """The reachable lateral interval at ``depth``, interpolated between
        the measured shallow and deep rows."""
        span = self.depth - self.shallow_depth
        if span < 1e-9:
            return self.deep_lateral_min, self.deep_lateral_max
        fraction = max(0.0, min(1.0, (depth - self.shallow_depth) / span))
        return (
            self.shallow_lateral_min
            + fraction * (self.deep_lateral_min - self.shallow_lateral_min),
            self.shallow_lateral_max
            + fraction * (self.deep_lateral_max - self.shallow_lateral_max),
        )

    def depth_of(self, point: Point3) -> float:
        """Offset of the table-frame ``point`` from the base along the approach
        direction, in m."""
        return (float(point.x) - self.base_x) * self.direction_x + (
            float(point.y) - self.base_y
        ) * self.direction_y

    def lateral_of(self, point: Point3) -> float:
        """Offset of the table-frame ``point`` from the base along the table
        edge (perpendicular to the approach direction), in m."""
        return -(float(point.x) - self.base_x) * self.direction_y + (
            float(point.y) - self.base_y
        ) * self.direction_x


@dataclass
class WipeTableMotion(BaseMotion):
    """Sweep the tool over a table surface, regulating contact force with the
    admittance controller, by building a single :class:`WipeGoal`.

    ``SPILL`` wipes a continuous serpentine over the whole surface; ``CRUMB``
    runs one +x gathering stroke per lane onto a shared collection point.
    Strokes are cut where they cross an obstacle so the tool lifts over it.
    With ``limit_to_reachable`` the arm's reach is measured once via IK and
    waypoints beyond it are dropped."""

    table: Table
    arm: Arms
    mode: WipeMode

    tool: Body | None = None
    """Sponge/tool body. Its footprint sets the lane spacing and margins."""

    tool_contact_frame: KinematicStructureEntity | None = None
    """Frame driven along the waypoints (e.g. a sponge's underside); defaults
    to the gripper tool frame."""

    lane_spacing: float | None = None
    surface_margin: float | None = None
    obstacle_margin: float | None = None
    """Distance between strokes / edge inset / obstacle inflation, in m.
    ``None`` derives them from the tool width."""

    stroke_sample_count: int = 2
    """Waypoints per stroke (>= 2); raise for non-flat surfaces or dense paths."""

    region: Rectangle | None = None
    """``(min_x, max_x, min_y, max_y)`` sub-rectangle to wipe; ``None`` = whole top."""

    limit_to_reachable: bool = False
    """Measure the reachable band once via IK and drop waypoints outside it.
    The robot must already face its wiping spot."""

    obstacles: list[Body] | None = None
    """Bodies on the table to wipe around; ``None`` uses ``table.objects``."""

    approach_height: float = 0.10
    """Height above the surface for approach/retract and lifts; clear obstacles."""

    desired_force: Vector3 = field(default_factory=lambda: Vector3(z=8.0))
    """Contact force the admittance balances, in the table frame."""

    contact_force_threshold: float = 3.0
    wipe_threshold: float = 0.03

    force_torque_reference_frame: KinematicStructureEntity | None = None
    """Frame the wrench is expressed in. The compensation node keeps it in the
    sensor frame, so set this to the sensor link; defaults to the tool tip."""

    avoid_collisions: bool = False
    """Run external collision avoidance in parallel with the wipe. Off by
    default so callers that add their own collision rules aren't doubled up."""

    verbose: bool = False
    """Print the world-frame wipe targets when the motion chart is built."""

    def __post_init__(self) -> None:
        width = _DEFAULT_TOOL_WIDTH
        if self.tool is not None and self.tool.has_collision():
            box = self.tool.collision.as_bounding_box_collection_in_frame(
                self.tool
            ).bounding_box()
            # Contact-face width: the middle extent (the smallest is the
            # thickness, the largest the stroke direction).
            width = sorted(
                [box.max_x - box.min_x, box.max_y - box.min_y, box.max_z - box.min_z]
            )[1]
        if self.lane_spacing is None:
            self.lane_spacing = width * (1.0 - _LANE_OVERLAP)
        if self.surface_margin is None:
            self.surface_margin = width / 2.0
        if self.obstacle_margin is None:
            self.obstacle_margin = width / 2.0 + _OBSTACLE_SAFETY

    def perform(self) -> None:
        return

    @property
    def _table_body(self) -> Body:
        return self.table.root

    @property
    def _tip_link(self) -> KinematicStructureEntity:
        if self.tool_contact_frame is not None:
            return self.tool_contact_frame
        return ViewManager.get_end_effector_view(self.arm, self.robot).tool_frame

    @property
    def _motion_chart(self) -> WipeGoal:
        tip_link = self._tip_link
        force_torque_node = ForceTorqueSymbolNode(
            topic_name=COMPENSATED_WRENCH_TOPIC,
            reference_frame=self.force_torque_reference_frame or tip_link,
            name="wipe_ft",
        )
        parallel_nodes: list[MotionStatechartNode] = []
        approach_rules: list[CollisionRule] = []
        contact_rules: list[CollisionRule] = []
        if self.avoid_collisions:
            # Weighted above the wipe so the QP is driven away from the table
            # rather than through it, even if a waypoint then stays unreached.
            parallel_nodes = [
                ExternalCollisionAvoidance(
                    robot=self.robot, weight=DefaultWeights.WEIGHT_MAX
                )
            ]
            approach_rules = self._approach_collision_rules()
            contact_rules = self._contact_collision_rules()
        goal = WipeGoal(
            name="WipeTable",
            segments=self._build_segments(),
            tip_link=tip_link,
            root_link=self.world.root,
            force_torque_node=force_torque_node,
            contact_force_threshold=self.contact_force_threshold,
            wipe_threshold=self.wipe_threshold,
            desired_force=self.desired_force,
            approach_height=self.approach_height,
            keep_tip_axis_aligned=True,
            parallel_nodes=parallel_nodes,
            approach_collision_rules=approach_rules,
            contact_collision_rules=contact_rules,
        )
        if self.verbose:
            self._print_targets(goal)
        return goal

    def _approach_collision_rules(self) -> list[CollisionRule]:
        """Avoid all external collisions; the tool is bolted to the arm, so its
        contact with the robot is never a collision."""
        rules: list[CollisionRule] = [AvoidExternalCollisions(robot=self.robot)]
        if self.tool is not None:
            rules.append(
                AllowCollisionBetweenGroups(
                    body_group_a=[self.tool],
                    body_group_b=list(self.robot.bodies_with_collision),
                )
            )
        return rules

    def _contact_collision_rules(self) -> list[CollisionRule]:
        """The approach rules plus the gripper and tool allowed onto the wiped
        surface -- the base stays table-avoided."""
        gripper = [
            body
            for body in self.world.bodies_with_collision
            if "hand_" in str(body.name)
        ]
        tool_group = gripper + ([self.tool] if self.tool is not None else [])
        return self._approach_collision_rules() + [
            AllowCollisionBetweenGroups(
                body_group_a=tool_group, body_group_b=[self._table_body]
            )
        ]

    def _print_targets(self, goal: WipeGoal) -> None:
        """Print, in the world frame, where the wipe drives ``tip_link``."""
        tip = goal.tip_link
        tip_now = self.world.compute_forward_kinematics_np(self.world.root, tip)[:3, 3]
        print(f"tip '{tip.name}' now (world): {np.round(tip_now, 3)}")
        for segment_index, waypoints in enumerate(goal.segments):
            if not waypoints:
                continue
            surface_T = self.world.compute_forward_kinematics_np(
                self.world.root, waypoints[0].reference_frame
            )

            def to_world(x: float, y: float, z: float) -> np.ndarray:
                return (surface_T @ np.array([x, y, z, 1.0]))[:3]

            first, last = waypoints[0], waypoints[-1]
            print(f"segment {segment_index} ({len(waypoints)} waypoints):")
            print(f"  approach (world): "
                  f"{np.round(to_world(float(first.x), float(first.y), float(first.z) + goal.approach_height), 3)}")
            print(f"  lower    (world): "
                  f"{np.round(to_world(float(first.x), float(first.y), float(first.z) - goal.lower_overshoot), 3)}"
                  f"  <- descends here unless force >= {goal.contact_force_threshold} N")
            for waypoint_index, waypoint in enumerate(waypoints):
                print(f"  waypoint {waypoint_index} (world): "
                      f"{np.round(to_world(float(waypoint.x), float(waypoint.y), float(waypoint.z)), 3)}")
            print(f"  retract  (world): "
                  f"{np.round(to_world(float(last.x), float(last.y), float(last.z) + goal.approach_height), 3)}")

    def _build_segments(self) -> list[WipeSegment]:
        min_x, max_x, min_y, max_y, top_z = self._wipe_extent()
        x_low = min_x + self.surface_margin
        x_high = max_x - self.surface_margin
        lane_y_values = self._lane_offsets(
            min_y + self.surface_margin, max_y - self.surface_margin
        )
        reach = (
            self._measure_reach_limit(
                x_low, x_high, lane_y_values[0], lane_y_values[-1], top_z
            )
            if self.limit_to_reachable
            else None
        )
        if reach is not None:
            # Start the serpentine at the corner nearest the base: least base
            # travel, and (approaching head-on) the shallowest row first.
            x_start, x_end = (
                (x_high, x_low)
                if abs(reach.base_x - x_high) < abs(reach.base_x - x_low)
                else (x_low, x_high)
            )
            if abs(reach.base_y - lane_y_values[-1]) < abs(reach.base_y - lane_y_values[0]):
                lane_y_values = list(reversed(lane_y_values))
        else:
            x_start, x_end = x_low, x_high
        footprints = self._obstacle_footprints()
        if self.mode == WipeMode.SPILL:
            segments = self._absorb_segments(
                x_start, x_end, lane_y_values, top_z, footprints, reach
            )
        else:
            segments = self._collect_segments(
                x_start, x_end, lane_y_values, top_z, footprints, reach
            )
        if reach is not None:
            waypoints = [waypoint for run in segments for waypoint in run]
            if waypoints:
                depths = [reach.depth_of(waypoint) for waypoint in waypoints]
                print(
                    f"reachability: {len(waypoints)} waypoints kept within the "
                    f"measured {reach.depth:.2f} m depth (waypoint depth "
                    f"{min(depths):.2f}-{max(depths):.2f} m)"
                )
            else:
                print(
                    f"reachability: NO waypoint within the measured "
                    f"{reach.depth:.2f} m depth -- move the base closer to the "
                    "wipe area."
                )
        return segments

    def _absorb_segments(
        self,
        x_start: float,
        x_end: float,
        lane_y_values: list[float],
        top_z: float,
        footprints: list[Rectangle],
        reach: ReachLimit | None,
    ) -> list[WipeSegment]:
        """One serpentine, cut wherever it hits an obstacle or leaves the
        reachable band. Cuts whose straight connector stays clear of every
        footprint are rejoined: the tool only lifts over obstacles, at reach
        clips it slides to the next row on the surface."""
        waypoints: list[Point3] = []
        for lane_index, lane_y in enumerate(lane_y_values):
            stroke_start, stroke_end = (
                (x_start, x_end) if lane_index % 2 == 0 else (x_end, x_start)
            )
            waypoints.extend(self._stroke_points(stroke_start, stroke_end, lane_y, top_z))
        segments: list[WipeSegment] = []
        for run in self._split_reachable(waypoints, footprints, reach):
            crosses = segments and any(
                _segment_crosses_rectangle(segments[-1][-1], run[0], footprint)
                for footprint in footprints
            )
            if segments and not crosses:
                segments[-1].extend(run)
            else:
                segments.append(run)
        return segments

    def _collect_segments(
        self,
        x_start: float,
        x_end: float,
        lane_y_values: list[float],
        top_z: float,
        footprints: list[Rectangle],
        reach: ReachLimit | None,
    ) -> list[WipeSegment]:
        """One +x stroke per lane, each funneled onto a shared collection point."""
        collection_y = 0.5 * (lane_y_values[0] + lane_y_values[-1])
        collection_point = Point3(
            x=x_end, y=collection_y, z=top_z, reference_frame=self._table_body
        )
        collection_clear = not self._is_obstructed(collection_point, footprints)
        segments: list[WipeSegment] = []
        for lane_y in lane_y_values:
            runs = self._split_reachable(
                self._stroke_points(x_start, x_end, lane_y, top_z), footprints, reach
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
                segments.append(run)
        return segments

    def _wipe_extent(self) -> tuple[float, float, float, float, float]:
        """Wipe bounds ``(min_x, max_x, min_y, max_y, top_z)`` in the table
        frame: the table's collision extent, clipped to ``region`` if set."""
        body = self._table_body
        boxes = list(body.collision.as_bounding_box_collection_in_frame(body))
        if not boxes:
            raise ValueError(f"Table {body.name} has no collision geometry.")
        min_x = min(box.min_x for box in boxes)
        max_x = max(box.max_x for box in boxes)
        min_y = min(box.min_y for box in boxes)
        max_y = max(box.max_y for box in boxes)
        top_z = max(box.max_z for box in boxes)
        if self.region is not None:
            region_min_x, region_max_x, region_min_y, region_max_y = self.region
            min_x, max_x = max(min_x, region_min_x), min(max_x, region_max_x)
            min_y, max_y = max(min_y, region_min_y), min(max_y, region_max_y)
        return min_x, max_x, min_y, max_y, top_z

    def _obstacle_footprints(self) -> list[Rectangle]:
        """Inflated xy footprints per obstacle; obstacles default to the bodies
        the table carries."""
        obstacles = (
            self.obstacles
            if self.obstacles is not None
            else [obj.root for obj in self.table.objects]
        )
        footprints: list[Rectangle] = []
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

    def _is_obstructed(self, point: Point3, footprints: list[Rectangle]) -> bool:
        x, y = float(point.x), float(point.y)
        return any(
            min_x <= x <= max_x and min_y <= y <= max_y
            for min_x, max_x, min_y, max_y in footprints
        )

    def _split_reachable(
        self,
        points: list[Point3],
        footprints: list[Rectangle],
        reach: ReachLimit | None,
    ) -> list[list[Point3]]:
        """Split into contiguous runs, dropping points inside an obstacle
        footprint or beyond the measured reach."""
        runs: list[list[Point3]] = []
        current: list[Point3] = []
        for point in points:
            if self._is_obstructed(point, footprints) or (
                reach is not None and not reach.contains(point)
            ):
                if current:
                    runs.append(current)
                    current = []
            else:
                current.append(point)
        if current:
            runs.append(current)
        return runs

    def _base_in_table_xy(self) -> tuple[float, float]:
        """The base position in the table's (horizontal) xy-plane."""
        table_T_world = np.linalg.inv(
            self.world.compute_forward_kinematics_np(self.world.root, self._table_body)
        )
        base_world = self.world.compute_forward_kinematics_np(
            self.world.root, self.robot.root
        )[:, 3]
        base_in_table = table_T_world @ base_world
        return float(base_in_table[0]), float(base_in_table[1])

    def _measure_reach_limit(
        self, x_low: float, x_high: float, y_low: float, y_high: float, top_z: float
    ) -> ReachLimit:
        """Measure via IK which trapezoid band of the wipe area the tool
        reaches: the depth is bisected straight ahead of the base, then the
        lateral bounds along the shallow and the deepest reachable row. Probes
        run under the wipe's contact collision rules, so blocked base
        stand-offs are part of the measurement, and with state-change
        callbacks paused, so neither RViz nor a synchronized world sees them."""
        if not self.world.is_controlled_connection_in_chain(
            self.robot.root, self._table_body
        ):
            raise UncontrolledRobotBaseError(robot=self.robot)
        base_x, base_y = self._base_in_table_xy()
        base = np.array([base_x, base_y])
        towards_centre = np.array([(x_low + x_high) / 2.0, (y_low + y_high) / 2.0]) - base
        if float(np.linalg.norm(towards_centre)) < 1e-9:
            raise ValueError("The robot base stands at the centre of the wipe area.")
        # Snapped to the dominant table axis, so the depth cut runs parallel
        # to the approached edge.
        if abs(towards_centre[0]) >= abs(towards_centre[1]):
            direction = np.array([math.copysign(1.0, towards_centre[0]), 0.0])
        else:
            direction = np.array([0.0, math.copysign(1.0, towards_centre[1])])
        perpendicular = np.array([-direction[1], direction[0]])
        corners = [
            np.array([corner_x, corner_y]) - base
            for corner_x in (x_low, x_high)
            for corner_y in (y_low, y_high)
        ]
        corner_depths = [float(corner @ direction) for corner in corners]
        corner_laterals = [float(corner @ perpendicular) for corner in corners]
        shallow, deepest = min(corner_depths), max(corner_depths)
        # Straight ahead of the base, clamped onto the area: the anchor every
        # bisection starts from.
        anchor = float(np.clip(0.0, min(corner_laterals), max(corner_laterals)))

        collision_manager = self.world.collision_manager
        collision_manager.clear_temporary_rules()
        for rule in self._contact_collision_rules():
            collision_manager.add_temporary_rule(rule)
        collision_manager.update_collision_matrix()
        # Only the ROS-adapter callbacks may be paused: internal ones (the
        # collision detector's state updater!) must keep running, or every
        # probe checks collisions against a stale robot pose and drives
        # through furniture.
        state_callbacks = [
            callback
            for callback in self.world.state.state_change_callbacks
            if type(callback).__module__.startswith("semantic_digital_twin.adapters.ros")
        ]
        for callback in state_callbacks:
            callback.pause()
        original_state = deepcopy(self.world.state._data)
        # Warm up into a wiping stance in two stages -- first turn the tool
        # down in place, then hover above the nearest point -- because pulling
        # position and orientation together out of the park pose bifurcates
        # into a sideways stance that cannot rotate out. The hover height must
        # clear every collision-avoidance activation zone or the QP wedges.
        table_T_world = np.linalg.inv(
            self.world.compute_forward_kinematics_np(self.world.root, self._table_body)
        )
        tip_world = self.world.compute_forward_kinematics_np(
            self.world.root, self._tip_link
        )[:, 3]
        tip_in_table = table_T_world @ tip_world
        self._tip_reaches(
            float(tip_in_table[0]), float(tip_in_table[1]), float(tip_in_table[2])
        )
        warm_up = base + shallow * direction + anchor * perpendicular
        self._tip_reaches(float(warm_up[0]), float(warm_up[1]), top_z + 0.25)

        def reaches(depth: float, lateral: float) -> bool:
            x, y = base + depth * direction + lateral * perpendicular
            probe = self._tip_reaches(float(x), float(y), top_z)
            if not probe.converged:
                return False
            # The QP happily walks around a free-standing table and reaches a
            # "deep" point from behind; the one-sided wipe never can, so the
            # base must have stayed on the robot's side.
            base_depth = (probe.base_x - base_x) * direction[0] + (
                probe.base_y - base_y
            ) * direction[1]
            return base_depth < shallow

        def lateral_bound(row_depth: float, limit: float) -> float:
            """Farthest reachable lateral offset toward ``limit`` along the row
            at ``row_depth``."""
            if reaches(row_depth, limit):
                return limit
            return self._bisect_boundary(
                anchor, limit, lambda lateral: reaches(row_depth, lateral)
            )

        if not reaches(shallow, anchor):
            depth = 0.0
            shallow_laterals = deep_laterals = (anchor, anchor)
        else:
            depth = (
                deepest
                if reaches(deepest, anchor)
                else self._bisect_boundary(
                    shallow, deepest, lambda probed: reaches(probed, anchor)
                )
            )
            shallow_laterals = (
                lateral_bound(shallow, min(corner_laterals)),
                lateral_bound(shallow, max(corner_laterals)),
            )
            # Measured again just inside the deepest row and clamped inside
            # the shallow bounds, so the interpolated band never widens with
            # depth (the anchor bisection alone finds the reach blob's pointy
            # extreme, reachable only straight ahead of the base).
            deep_row = max(shallow, depth - _REACH_PROBE_TOLERANCE)
            deep_laterals = (
                max(lateral_bound(deep_row, min(corner_laterals)), shallow_laterals[0]),
                min(lateral_bound(deep_row, max(corner_laterals)), shallow_laterals[1]),
            )

        self.world.state._data[:] = original_state
        for callback in state_callbacks:
            callback.resume()
        self.world.notify_state_change()
        collision_manager.clear_temporary_rules()
        collision_manager.update_collision_matrix()
        if depth:
            print(f"reach measured via IK: depth {shallow:.2f} to {depth:.2f} m, "
                  f"lateral {shallow_laterals[0]:+.2f}..{shallow_laterals[1]:+.2f} m "
                  f"at the near row, {deep_laterals[0]:+.2f}..{deep_laterals[1]:+.2f} m "
                  "at the deep row")
        else:
            print(f"reach measured via IK: not even the nearest point of the wipe "
                  f"area ({shallow:.2f} m out) is reachable -- move the base to face it.")
        return ReachLimit(
            base_x=base_x,
            base_y=base_y,
            direction_x=float(direction[0]),
            direction_y=float(direction[1]),
            shallow_depth=shallow,
            depth=depth,
            shallow_lateral_min=shallow_laterals[0],
            shallow_lateral_max=shallow_laterals[1],
            deep_lateral_min=deep_laterals[0],
            deep_lateral_max=deep_laterals[1],
        )

    def _bisect_boundary(
        self, reachable: float, unreachable: float, reaches: Callable[[float], bool]
    ) -> float:
        """The last reachable value between a probe parameter known to reach
        and one known not to, bisected to the probe tolerance."""
        low, high = reachable, unreachable
        while abs(high - low) > _REACH_PROBE_TOLERANCE:
            middle = (low + high) / 2.0
            if reaches(middle):
                low = middle
            else:
                high = middle
        return low

    def _tip_reaches(self, x: float, y: float, z: float) -> ReachProbeResult:
        """One IK probe under the active collision rules: drive the tip onto
        the table-frame point with the tool pointing down, and report whether
        both converge plus the base stance that achieved it. Probes run
        sequentially, each from the previous stance (cold from the park pose
        the QP wedges in local minima); the caller owns state restore."""
        tip = self._tip_link
        position = CartesianPosition(
            root_link=self.world.root,
            tip_link=tip,
            goal_point=Point3(x=x, y=y, z=z, reference_frame=self._table_body),
        )
        pointing_down = AlignTipAxis(
            root_link=self.world.root,
            tip_link=tip,
            tip_axis=Vector3.Z(reference_frame=tip),
            root_axis_goal=Vector3(x=0, y=0, z=-1, reference_frame=self.world.root),
            reference_velocity=0.2,
            # Tighter tilts count as sideways pokes the wipe cannot press from.
            threshold=0.35,
            weight=0.2 * DefaultWeights.WEIGHT_ABOVE_CA,
        )
        statechart = MotionStatechart()
        statechart.add_node(
            ExternalCollisionAvoidance(
                robot=self.robot, weight=DefaultWeights.WEIGHT_MAX
            )
        )
        statechart.add_node(position)
        statechart.add_node(pointing_down)
        end = EndMotion()
        end.start_condition = trinary_logic_and(
            position.observation_variable, pointing_down.observation_variable
        )
        statechart.add_node(end)
        executor = Executor(
            context=MotionStatechartContext(
                world=self.world,
                qp_controller_config=QPControllerConfig(
                    target_frequency=50, prediction_horizon=4, verbose=False
                ),
            ),
        )
        executor.compile(statechart)
        goal_world = (
            self.world.compute_forward_kinematics_np(self.world.root, self._table_body)
            @ np.array([x, y, z, 1.0])
        )[:3]
        reached = False
        best_score = math.inf
        stalled_ticks = 0
        try:
            for _ in range(1000):
                executor.tick()
                if statechart.is_end_motion():
                    reached = True
                    break
                tip_transform = self.world.compute_forward_kinematics_np(
                    self.world.root, tip
                )
                error = float(np.linalg.norm(tip_transform[:3, 3] - goal_world))
                tilt = math.acos(float(np.clip(-tip_transform[2, 2], -1.0, 1.0)))
                # No 5 mm of combined progress within 150 ticks: wedged in a
                # local minimum or crawling along the reach boundary.
                score = error + 0.3 * tilt
                if score < best_score - 0.005:
                    best_score = score
                    stalled_ticks = 0
                else:
                    stalled_ticks += 1
                    if stalled_ticks >= 150:
                        break
            base_end_x, base_end_y = self._base_in_table_xy()
        finally:
            statechart.cleanup_nodes(context=executor.context)
            executor.context.cleanup()
        return ReachProbeResult(
            converged=reached, base_x=base_end_x, base_y=base_end_y
        )

    def _stroke_points(
        self, x_start: float, x_end: float, lane_y: float, top_z: float
    ) -> list[Point3]:
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

    def _lane_offsets(self, start: float, end: float) -> list[float]:
        """Lane y-values from ``start`` to ``end``, spaced by at most ``lane_spacing``."""
        span = end - start
        if span <= 0.0:
            return [start]
        lane_count = max(1, math.ceil(span / self.lane_spacing))
        return [start + min(i * self.lane_spacing, span) for i in range(lane_count + 1)]
