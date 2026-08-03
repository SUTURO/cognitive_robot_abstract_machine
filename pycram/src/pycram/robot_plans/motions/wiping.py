from __future__ import annotations

import math
from dataclasses import dataclass, field

from giskardpy.motion_statechart.data_types import DefaultWeights
from giskardpy.motion_statechart.goals.collision_avoidance import (
    ExternalCollisionAvoidance,
)
from giskardpy.motion_statechart.goals.wipe_goals import WipeGoal, WipeSegment
from giskardpy.motion_statechart.graph_node import MotionStatechartNode
from giskardpy.motion_statechart.ros2_nodes.force_torque_monitor import (
    ForceTorqueSymbolNode,
)
from giskardpy.ros2_tools.wrench_compensation_node import COMPENSATED_WRENCH_TOPIC
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionBetweenGroups,
    AvoidExternalCollisions,
    CollisionRule,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import Table
from semantic_digital_twin.spatial_types import Point3, Vector3
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import BoundingBox
from semantic_digital_twin.world_description.world_entity import (
    Body,
    KinematicStructureEntity,
)

from pycram.datastructures.enums import Arms, WipeMode
from pycram.robot_plans.motions.base import BaseMotion
from pycram.robot_plans.motions.wipe_coverage import Reach
from pycram.view_manager import ViewManager

# Lane spacing and surface margin are derived from the tool's contact width (a
# default when no tool is given): lanes overlap by this fraction of it.
_DEFAULT_TOOL_WIDTH = 0.07
_LANE_OVERLAP = 0.3


@dataclass(frozen=True)
class ForceControl:
    """The contact force to press with and where it is measured."""

    desired_force: Vector3 = field(default_factory=lambda: Vector3(z=8.0))
    """Contact force the admittance balances, in the table frame."""

    sensor_frame: KinematicStructureEntity | None = None
    """Frame the compensated wrench is measured in (the FT sensor link); the
    admittance rotates it into the table frame. ``None`` uses the tool tip, which
    is only correct when the sensor and tip share an orientation."""


@dataclass
class WipeTableMotion(BaseMotion):
    """Sweep the tool over a cleared table surface, regulating contact force with
    the admittance controller, by building a single :class:`WipeGoal`.

    ``SPILL`` wipes a continuous serpentine over the surface; ``CRUMB`` pushes
    every lane toward the +x edge and then sweeps once along that edge to gather
    the piles into one corner. A ``reach`` from the coverage planner tapers each
    lane to the arm's reach from where the base can stand."""

    table: Table
    """Table whose top surface is wiped (assumed cleared of clutter)."""

    arm: Arms
    """Which arm performs the wipe."""

    mode: WipeMode
    """``SPILL`` (serpentine) or ``CRUMB`` (gather to a corner)."""

    tool: Body | None = None
    """Sponge/tool body. Its footprint sets the lane spacing and margin."""

    tool_contact_frame: KinematicStructureEntity | None = None
    """Frame driven along the waypoints; defaults to the gripper tool frame."""

    region: tuple[float, float, float, float] | None = None
    """``(min_x, max_x, min_y, max_y)`` sub-rectangle to wipe; ``None`` = whole top."""

    reach: Reach | None = None
    """Where the base can stand and how far the arm reaches, from the coverage
    planner; tapers lanes past the standing line. ``None`` = no reach limit."""

    force: ForceControl = field(default_factory=ForceControl)
    """The press force and the frame its wrench is measured in."""

    avoid_collisions: bool = False
    """Run external collision avoidance in parallel with the wipe."""

    lane_spacing: float | None = None
    """Distance between adjacent strokes, in m. ``None`` derives it from the tool."""

    surface_margin: float | None = None
    """Inset of the wipe area from the table edge, in m. ``None`` derives it."""

    stroke_sample_count: int = 2
    """Waypoints per stroke (>= 2); raise for a finer reach taper."""

    def __post_init__(self) -> None:
        width = _DEFAULT_TOOL_WIDTH
        if self.tool is not None and self.tool.has_collision():
            box = self.tool.collision.as_bounding_box_collection_in_frame(
                self.tool
            ).bounding_box()
            # The wipe holds the tool's long side across the strokes, so the
            # coverage per lane is the largest horizontal extent.
            width = max(box.dimensions)
        if self.lane_spacing is None:
            self.lane_spacing = width * (1.0 - _LANE_OVERLAP)
        if self.surface_margin is None:
            self.surface_margin = width / 2.0

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
            reference_frame=self.force.sensor_frame or tip_link,
            name="wipe_ft",
        )
        parallel_nodes: list[MotionStatechartNode] = []
        approach_rules: list[CollisionRule] = []
        contact_rules: list[CollisionRule] = []
        if self.avoid_collisions:
            parallel_nodes = [
                ExternalCollisionAvoidance(
                    robot=self.robot, weight=DefaultWeights.WEIGHT_MAX
                )
            ]
            approach_rules = self._approach_collision_rules()
            contact_rules = self._contact_collision_rules()
        return WipeGoal(
            name="WipeTable",
            segments=self._build_segments(),
            tip_link=tip_link,
            root_link=self.world.root,
            force_torque_node=force_torque_node,
            desired_force=self.force.desired_force,
            tool_heading=self._tool_heading(),
            parallel_nodes=parallel_nodes,
            approach_collision_rules=approach_rules,
            contact_collision_rules=contact_rules,
        )

    def _build_segments(self) -> list[WipeSegment]:
        extent = self._wipe_extent()
        top_z = extent.max_z
        x_low = extent.min_x + self.surface_margin
        x_high = extent.max_x - self.surface_margin
        lane_y_values = self._lane_offsets(
            extent.min_y + self.surface_margin, extent.max_y - self.surface_margin
        )
        lanes = [self._stroke_points(x_low, x_high, y, top_z) for y in lane_y_values]
        if self.reach is not None:
            lanes = [self._within_arm_reach(lane) for lane in lanes]
        if self.mode == WipeMode.SPILL:
            return self._absorb_segments(lanes)
        return self._collect_segments(lanes)

    def _within_arm_reach(self, lane: list[Point3]) -> list[Point3]:
        """The waypoints of ``lane`` within the arm's reach of the base's standing
        line: full strokes where the base stands beside them, tapering to nothing
        where only the arm's sideways reach can get there."""
        return [
            point
            for point in lane
            if self._distance_to_reach_segment(point) <= self.reach.radius
        ]

    def _distance_to_reach_segment(self, point: Point3) -> float:
        """Table-frame xy distance from ``point`` to the base's standing line."""
        (start_x, start_y), (end_x, end_y) = self.reach.segment
        point_x, point_y = float(point.x), float(point.y)
        segment_x, segment_y = end_x - start_x, end_y - start_y
        length_squared = segment_x * segment_x + segment_y * segment_y
        if length_squared == 0.0:
            return math.hypot(point_x - start_x, point_y - start_y)
        fraction = max(
            0.0,
            min(
                1.0,
                ((point_x - start_x) * segment_x + (point_y - start_y) * segment_y)
                / length_squared,
            ),
        )
        return math.hypot(
            point_x - (start_x + fraction * segment_x),
            point_y - (start_y + fraction * segment_y),
        )

    def _absorb_segments(self, lanes: list[list[Point3]]) -> list[WipeSegment]:
        """One serpentine over the lanes: adjacent lanes run in opposite
        x-directions and the tool slides straight from one to the next."""
        waypoints: list[Point3] = []
        for lane_index, lane in enumerate(lanes):
            waypoints.extend(lane if lane_index % 2 == 0 else list(reversed(lane)))
        return [waypoints] if waypoints else []

    def _collect_segments(self, lanes: list[list[Point3]]) -> list[WipeSegment]:
        """Every lane pushes its crumbs toward the +x edge, then one final sweep
        runs along that edge through every lane's pile, gathering them into one
        corner. The tool lifts between the lanes and before the sweep."""
        lanes = [lane for lane in lanes if lane]
        piles = [lane[-1] for lane in lanes]
        segments: list[WipeSegment] = list(lanes)
        if len(piles) >= 2:
            segments.append(piles)
        return segments

    def _wipe_extent(self) -> BoundingBox:
        """The table's collision extent in its own frame, clipped to ``region``
        if set. The surface height is its ``max_z``."""
        body = self._table_body
        collection = body.collision.as_bounding_box_collection_in_frame(body)
        if not collection.bounding_boxes:
            raise ValueError(f"Table {body.name} has no collision geometry.")
        extent = collection.bounding_box()
        if self.region is not None:
            min_x, max_x, min_y, max_y = self.region
            extent = BoundingBox(
                max(extent.min_x, min_x),
                max(extent.min_y, min_y),
                extent.min_z,
                min(extent.max_x, max_x),
                min(extent.max_y, max_y),
                extent.max_z,
                extent.origin,
            )
        return extent

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

    def _tool_heading(self) -> Vector3:
        """World-frame direction of the table's cross-stroke axis (its y-axis,
        pointing away from the robot): the wipe holds the tool's finger axis along
        it, so an elongated tool covers the lane with its long side."""
        extent = self._wipe_extent()
        table_T_base = self.world.compute_forward_kinematics_np(
            self._table_body, self.robot.root
        )
        base_y = float(table_T_base[1, 3])
        sign = math.copysign(1.0, (extent.min_y + extent.max_y) / 2.0 - base_y)
        world_direction = sign * self.world.compute_forward_kinematics_np(
            self.world.root, self._table_body
        )[:3, 1]
        return Vector3.from_iterable(world_direction, reference_frame=self.world.root)

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
        return self._approach_collision_rules() + [
            AllowCollisionBetweenGroups(
                body_group_a=self._gripper_and_tool_bodies(),
                body_group_b=[self._table_body],
            )
        ]

    def _gripper_and_tool_bodies(self) -> list[Body]:
        """The gripper's own collision bodies plus the tool: the parts that may
        rest on the wiped surface while pressing."""
        gripper = ViewManager.get_end_effector_view(self.arm, self.robot)
        bodies = [
            entity
            for entity in self.world.get_kinematic_structure_entities_of_branch(
                gripper.root
            )
            if isinstance(entity, Body) and entity.has_collision()
        ]
        if self.tool is not None:
            bodies.append(self.tool)
        return bodies
