from __future__ import annotations

import math
from dataclasses import dataclass, field

from giskardpy.motion_statechart.data_types import DefaultWeights
from giskardpy.motion_statechart.goals.collision_avoidance import (
    ExternalCollisionAvoidance,
)
from giskardpy.motion_statechart.goals.wipe_goals import (
    WipeCollision,
    WipeGoal,
    WipeSegment,
)
from giskardpy.motion_statechart.ros2_nodes.force_torque_monitor import (
    ForceTorqueSymbolNode,
)
from giskardpy.ros2_tools.wrench_compensation_node import COMPENSATED_WRENCH_TOPIC
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionBetweenGroups,
    AvoidExternalCollisions,
    CollisionRule,
)
from semantic_digital_twin.robots.abstract_robot import ForceTorqueSensor
from semantic_digital_twin.semantic_annotations.semantic_annotations import Table
from semantic_digital_twin.spatial_types import Point3, Vector3
from semantic_digital_twin.world_description.geometry import BoundingBox
from semantic_digital_twin.world_description.world_entity import (
    Body,
    KinematicStructureEntity,
)

from pycram.datastructures.enums import Arms, WipeMode
from pycram.robot_plans.motions.base import BaseMotion
from pycram.robot_plans.motions.wipe_coverage import Reach
from pycram.view_manager import ViewManager


class MissingForceTorqueSensorError(Exception):
    """Raised when the robot has no force/torque sensor annotation, so the wipe
    has no frame to measure its contact wrench in."""

    def __init__(self, robot: object) -> None:
        super().__init__(
            f"Robot {robot} has no ForceTorqueSensor annotation; the wipe cannot "
            "regulate contact force."
        )


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

    tool: Body
    """Sponge/tool body held by the gripper. Its frame follows the waypoints and
    its footprint sets the lane spacing and margin."""

    region: tuple[float, float, float, float] | None = None
    """``(min_x, max_x, min_y, max_y)`` sub-rectangle to wipe; ``None`` = whole top."""

    reach: Reach | None = None
    """Where the base can stand and how far the arm reaches, from the coverage
    planner; tapers lanes past the standing line. ``None`` = no reach limit."""

    desired_force: Vector3 = field(default_factory=lambda: Vector3(z=8.0))
    """Contact force the admittance balances, in the table frame."""

    avoid_collisions: bool = False
    """Run external collision avoidance in parallel with the wipe."""

    def perform(self) -> None:
        return

    @property
    def _table_body(self) -> Body:
        return self.table.root

    @property
    def _tool_width(self) -> float:
        """The tool's largest horizontal extent: the wipe holds its long side
        across the strokes, so this is the coverage one lane provides."""
        box = self.tool.collision.as_bounding_box_collection_in_frame(
            self.tool
        ).bounding_box()
        return max(box.dimensions)

    @property
    def lane_spacing(self) -> float:
        """Half the tool width, so adjacent lanes overlap by 50% and every
        surface point is covered by at least two strokes."""
        return self._tool_width / 2.0

    @property
    def surface_margin(self) -> float:
        """Half the tool width, so the tool's edge reaches the table edge while
        its centre stays inset."""
        return self._tool_width / 2.0

    @property
    def _force_torque_frame(self) -> KinematicStructureEntity:
        """The frame the compensated wrench is measured in, taken from the
        robot's force/torque sensor annotation."""
        for sensor in self.robot.sensors:
            if isinstance(sensor, ForceTorqueSensor):
                return sensor.root
        raise MissingForceTorqueSensorError(self.robot.name)

    @property
    def _motion_chart(self) -> WipeGoal:
        force_torque_node = ForceTorqueSymbolNode(
            topic_name=COMPENSATED_WRENCH_TOPIC,
            reference_frame=self._force_torque_frame,
            name="wipe_ft",
        )
        collision = None
        if self.avoid_collisions:
            collision = WipeCollision(
                avoidance=ExternalCollisionAvoidance(
                    robot=self.robot, weight=DefaultWeights.WEIGHT_MAX
                ),
                approach_rules=self._approach_collision_rules(),
                contact_rules=self._contact_collision_rules(),
            )
        return WipeGoal(
            name="WipeTable",
            segments=self._build_segments(),
            tip_link=self.tool,
            root_link=self.world.root,
            force_torque_node=force_torque_node,
            desired_force=self.desired_force,
            collision=collision,
        )

    def _build_segments(self) -> list[WipeSegment]:
        extent = self._wipe_extent()
        top_z = extent.max_z
        margin = self.surface_margin
        x_low = extent.min_x + margin
        x_high = extent.max_x - margin
        lane_y_values = self._lane_offsets(
            extent.min_y + margin, extent.max_y - margin
        )
        lanes = [self._stroke_points(x_low, x_high, y, top_z) for y in lane_y_values]
        if self.reach is not None:
            lanes = [
                [
                    point
                    for point in lane
                    if self._distance_to_reach_segment(point) <= self.reach.radius
                ]
                for lane in lanes
            ]
        if self.mode == WipeMode.SPILL:
            return self._absorb_segments(lanes)
        return self._collect_segments(lanes)

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
        """Waypoints along one stroke, sampled at the lane spacing so the reach
        taper and coverage share a uniform grid."""
        count = max(2, math.ceil(abs(x_end - x_start) / self.lane_spacing) + 1)
        return [
            Point3(
                x=x_start + i / (count - 1) * (x_end - x_start),
                y=lane_y,
                z=top_z,
                reference_frame=self._table_body,
            )
            for i in range(count)
        ]

    def _lane_offsets(self, start: float, end: float) -> list[float]:
        """Lane y-values evenly spanning ``start`` to ``end`` with a gap of at
        most ``lane_spacing`` -- both ends included exactly once."""
        span = end - start
        if span <= 0.0:
            return [start]
        lane_count = max(1, math.ceil(span / self.lane_spacing))
        return [start + i * span / lane_count for i in range(lane_count + 1)]

    def _approach_collision_rules(self) -> list[CollisionRule]:
        """Avoid all external collisions; the tool is bolted to the arm, so its
        contact with the robot is never a collision."""
        return [
            AvoidExternalCollisions(robot=self.robot),
            AllowCollisionBetweenGroups(
                body_group_a=[self.tool],
                body_group_b=list(self.robot.bodies_with_collision),
            ),
        ]

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
        bodies.append(self.tool)
        return bodies
