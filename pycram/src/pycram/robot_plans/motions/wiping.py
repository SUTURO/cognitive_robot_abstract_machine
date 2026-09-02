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
    ForceTorqueSensorUpdater,
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
from semantic_digital_twin.world_description.world_entity import Body

from pycram.datastructures.enums import Arms, WipeMode
from pycram.robot_plans.motions.base import BaseMotion
from pycram.robot_plans.motions.wipe_coverage import Reach, WipeRegion
from pycram.view_manager import ViewManager


class TableWithoutCollisionGeometryError(Exception):
    """Raised when a table has no collision geometry, so the wipe cannot tell where
    its surface is."""

    def __init__(self, table: Table) -> None:
        super().__init__(f"Table {table.root.name} has no collision geometry.")


@dataclass
class WipeTableMotion(BaseMotion):
    """Sweep the tool over a cleared table surface, regulating contact force with the
    admittance controller, by building a single :class:`WipeGoal`.

    ``SPILL`` wipes a continuous serpentine over the surface; ``CRUMB`` pushes every
    lane toward the +x edge and then sweeps once along that edge to gather the piles
    into one corner."""

    table: Table
    """Table whose top surface is wiped (assumed cleared of clutter)."""

    arm: Arms
    """Which arm performs the wipe."""

    mode: WipeMode
    """``SPILL`` (serpentine) or ``CRUMB`` (gather to a corner)."""

    tool: Body
    """Sponge/tool body held by the gripper. Its frame follows the waypoints and its
    footprint sets the lane spacing and margin."""

    region: WipeRegion | None = None
    """Sub-rectangle of the top to wipe. ``None`` wipes the whole top."""

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
    def _tool_width(self) -> float:
        """The tool's largest horizontal extent: the wipe holds its long side across
        the strokes, so this is the coverage one lane provides."""
        box = self.tool.collision.as_bounding_box_collection_in_frame(
            self.tool
        ).bounding_box()
        return max(box.dimensions)

    @property
    def lane_spacing(self) -> float:
        """Half the tool width, so adjacent lanes overlap by 50% and every surface
        point is covered by at least two strokes."""
        return self._tool_width / 2.0

    @property
    def surface_margin(self) -> float:
        """Half the tool width, so the tool's edge reaches the table edge while its
        centre stays inset."""
        return self._tool_width / 2.0

    @property
    def _motion_chart(self) -> WipeGoal:
        wrench_source = ForceTorqueSensorUpdater(
            topic_name=COMPENSATED_WRENCH_TOPIC,
            reference_frame=ForceTorqueSensor.for_tip(self.world, self.tool).root,
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
            wrench_source=wrench_source,
            desired_force=self.desired_force,
            collision=collision,
        )

    def _build_segments(self) -> list[WipeSegment]:
        """The strokes to run, in order, each in the table's frame."""
        extent = self._wipe_extent()
        margin = self.surface_margin
        lanes = [
            self._stroke_points(
                extent.min_x + margin, extent.max_x - margin, lane_y, extent.max_z
            )
            for lane_y in self._lane_offsets(
                extent.min_y + margin, extent.max_y - margin
            )
        ]
        if self.reach is not None:
            lanes = [
                [point for point in lane if self.reach.covers(point)] for lane in lanes
            ]
        lanes = [lane for lane in lanes if lane]

        if self.mode is WipeMode.SPILL:
            # One serpentine: adjacent lanes run in opposite x-directions, so the tool
            # slides straight from one into the next.
            serpentine = [
                point
                for lane_index, lane in enumerate(lanes)
                for point in (lane if lane_index % 2 == 0 else reversed(lane))
            ]
            return [serpentine] if serpentine else []

        # Every lane pushes its crumbs toward the +x edge, then one final sweep runs
        # along that edge through every lane's pile, gathering them into one corner.
        piles = [lane[-1] for lane in lanes]
        return lanes + [piles] if len(piles) >= 2 else lanes

    def _wipe_extent(self) -> BoundingBox:
        """The table's collision extent in its own frame, clipped to :attr:`region` if
        set. The surface height is its ``max_z``."""
        table_body = self.table.root
        collection = table_body.collision.as_bounding_box_collection_in_frame(
            table_body
        )
        if not collection.bounding_boxes:
            raise TableWithoutCollisionGeometryError(self.table)
        extent = collection.bounding_box()
        if self.region is None:
            return extent
        return self.region.clip(extent)

    def _stroke_points(
        self, x_start: float, x_end: float, lane_y: float, top_z: float
    ) -> list[Point3]:
        """Waypoints along one stroke, sampled at the lane spacing so the reach taper
        and the coverage share a uniform grid."""
        count = max(2, math.ceil(abs(x_end - x_start) / self.lane_spacing) + 1)
        return [
            Point3(
                x=x_start + index / (count - 1) * (x_end - x_start),
                y=lane_y,
                z=top_z,
                reference_frame=self.table.root,
            )
            for index in range(count)
        ]

    def _lane_offsets(self, start: float, end: float) -> list[float]:
        """Lane y-values evenly spanning ``start`` to ``end`` with a gap of at most
        :attr:`lane_spacing` -- both ends included exactly once."""
        span = end - start
        if span <= 0.0:
            return [start]
        lane_count = max(1, math.ceil(span / self.lane_spacing))
        return [start + index * span / lane_count for index in range(lane_count + 1)]

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
        """The approach rules plus the gripper's own collision bodies and the tool --
        the parts that rest on the surface while pressing -- allowed onto the wiped
        surface. The base stays table-avoided."""
        gripper = ViewManager.get_end_effector_view(self.arm, self.robot)
        pressing_bodies = [
            entity
            for entity in self.world.get_kinematic_structure_entities_of_branch(
                gripper.root
            )
            if isinstance(entity, Body) and entity.has_collision()
        ]
        return self._approach_collision_rules() + [
            AllowCollisionBetweenGroups(
                body_group_a=pressing_bodies + [self.tool],
                body_group_b=[self.table.root],
            )
        ]
