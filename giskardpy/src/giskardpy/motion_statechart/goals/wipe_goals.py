from __future__ import annotations

from dataclasses import dataclass, field

from semantic_digital_twin.collision_checking.collision_rules import CollisionRule
from semantic_digital_twin.spatial_types import Point3, Vector3
from semantic_digital_twin.world_description.world_entity import (
    KinematicStructureEntity,
)

from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.data_types import DefaultWeights
from giskardpy.motion_statechart.goals.collision_avoidance import (
    UpdateTemporaryCollisionRules,
)
from giskardpy.motion_statechart.goals.templates import Parallel, Sequence
from giskardpy.motion_statechart.graph_node import (
    MotionStatechartNode,
    NodeArtifacts,
)
from giskardpy.motion_statechart.ros2_nodes.force_torque_monitor import (
    ForceTorqueSymbolNode,
)
from giskardpy.motion_statechart.tasks.admittance_tasks import (
    AdmittanceCartesianPosition,
    LowerUntilContact,
)
from giskardpy.motion_statechart.tasks.align_planes import AlignPlanes
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPosition


WipeSegment = list[Point3]
"""The waypoints of one uninterrupted stroke run; the tool lifts between segments."""

_WEIGHT = DefaultWeights.WEIGHT_ABOVE_CA
_APPROACH_HEIGHT = 0.10
_LOWER_OVERSHOOT = 0.05
_CONTACT_FORCE_THRESHOLD = 3.0
_WIPE_THRESHOLD = 0.03
_APPROACH_VELOCITY = 0.10
_LOWER_VELOCITY = 0.03
_WIPE_VELOCITY = 0.10
_ALIGN_VELOCITY = 0.4
# z near-critical against a firm surface (~2 kN/m contact, unit virtual mass) so
# the press settles instead of bouncing; x/y soft so the strokes yield sideways.
_DAMPING = Vector3(x=20.0, y=20.0, z=100.0)


@dataclass(eq=False, repr=False)
class WipeGoal(Parallel):
    """Table-wiping goal. Each segment expands to ``CartesianPosition(approach)
    -> LowerUntilContact -> AdmittanceCartesianPosition per waypoint ->
    CartesianPosition(retract)``; these run as one inner sequence, in parallel
    with the force-torque source, the tool-down/heading alignment, and the
    collision nodes. The goal completes when that stroke sequence completes.
    Pass segments whose waypoints are reachable from the current base."""

    segments: list[WipeSegment] = field(kw_only=True)
    """Ordered waypoint runs to wipe; the tool lifts between runs."""

    tip_link: KinematicStructureEntity = field(kw_only=True)
    """End link that follows the waypoints (e.g. the gripper tool frame)."""

    root_link: KinematicStructureEntity = field(kw_only=True)
    """Root of the kinematic chain. Usually ``world.root``."""

    force_torque_node: ForceTorqueSymbolNode = field(kw_only=True)
    """Wrench source shared by ``LowerUntilContact`` and the admittance tasks.
    Owned and added to the chart by this goal."""

    desired_force: Vector3 | None = field(default=None, kw_only=True)
    """Contact force the admittance balances, forwarded to every wipe task."""

    tool_heading: Vector3 | None = field(default=None, kw_only=True)
    """Root-frame direction the tip's +X axis (the finger/long-tool axis) is held
    along; locks the yaw so an elongated tool lies across the strokes. ``None``
    leaves the yaw to the IK."""

    parallel_nodes: list[MotionStatechartNode] = field(
        default_factory=list, kw_only=True
    )
    """Extra nodes running in parallel with the strokes (e.g. collision
    avoidance), ended when the strokes finish."""

    approach_collision_rules: list[CollisionRule] = field(
        default_factory=list, kw_only=True
    )
    """Collision rules applied at the start of every segment (e.g. the gripper
    avoids the surface so it descends from above). Empty = matrix unchanged."""

    contact_collision_rules: list[CollisionRule] = field(
        default_factory=list, kw_only=True
    )
    """Collision rules applied before every contact-seeking descent (e.g. suspend
    gripper-vs-surface avoidance). Empty = matrix unchanged."""

    _strokes: Sequence | None = field(default=None, init=False, repr=False)
    """Inner sequence of approach/lower/wipe/retract tasks; the goal's
    observation tracks it, so parallel nodes cannot delay the end."""

    def expand(self, context: MotionStatechartContext) -> None:
        motion_nodes: list[MotionStatechartNode] = []
        for segment_index, waypoints in enumerate(self.segments):
            if waypoints:
                motion_nodes.extend(self._segment_nodes(segment_index, waypoints))
        if not motion_nodes:
            return

        self.add_node(self.force_torque_node)
        self._strokes = Sequence(name=f"{self.name}/strokes", nodes=motion_nodes)
        self.add_node(self._strokes)

        alignments = [
            AlignPlanes(
                name=f"{self.name}/align_tip",
                root_link=self.root_link,
                tip_link=self.tip_link,
                tip_normal=Vector3.Z(reference_frame=self.tip_link),
                goal_normal=Vector3(x=0, y=0, z=-1, reference_frame=self.root_link),
                weight=0.2 * _WEIGHT,
                reference_velocity=_ALIGN_VELOCITY,
            )
        ]
        if self.tool_heading is not None:
            alignments.append(
                AlignPlanes(
                    name=f"{self.name}/align_heading",
                    root_link=self.root_link,
                    tip_link=self.tip_link,
                    tip_normal=Vector3.X(reference_frame=self.tip_link),
                    goal_normal=self.tool_heading,
                    weight=0.2 * _WEIGHT,
                    reference_velocity=_ALIGN_VELOCITY,
                )
            )
        for node in alignments + self.parallel_nodes:
            self.add_node(node)
            node.end_condition = self._strokes.observation_variable

    def build(self, context: MotionStatechartContext) -> NodeArtifacts:
        if self._strokes is None:
            return NodeArtifacts()
        return NodeArtifacts(observation=self._strokes.observation_variable)

    def _segment_nodes(
        self, segment_index: int, waypoints: list[Point3]
    ) -> list[MotionStatechartNode]:
        segment_name = f"{self.name}/segment{segment_index}"
        first, last = waypoints[0], waypoints[-1]
        reference_frame = first.reference_frame

        nodes: list[MotionStatechartNode] = []
        if self.approach_collision_rules:
            nodes.append(
                UpdateTemporaryCollisionRules(
                    name=f"{segment_name}/approach_collision",
                    temporary_rules=self.approach_collision_rules,
                )
            )
        nodes.append(
            CartesianPosition(
                name=f"{segment_name}/approach",
                root_link=self.root_link,
                tip_link=self.tip_link,
                goal_point=Point3(
                    x=first.x, y=first.y, z=first.z + _APPROACH_HEIGHT,
                    reference_frame=reference_frame,
                ),
                reference_velocity=_APPROACH_VELOCITY,
                weight=_WEIGHT,
            )
        )
        if self.contact_collision_rules:
            nodes.append(
                UpdateTemporaryCollisionRules(
                    name=f"{segment_name}/contact_collision",
                    temporary_rules=self.contact_collision_rules,
                )
            )
        nodes.append(
            LowerUntilContact(
                name=f"{segment_name}/lower",
                root_link=self.root_link,
                tip_link=self.tip_link,
                goal_point=Point3(
                    x=first.x, y=first.y, z=first.z - _LOWER_OVERSHOOT,
                    reference_frame=reference_frame,
                ),
                ft_node=self.force_torque_node,
                force_threshold=_CONTACT_FORCE_THRESHOLD,
                reference_velocity=_LOWER_VELOCITY,
                weight=_WEIGHT,
            )
        )
        nodes.append(
            Sequence(
                name=f"{segment_name}/wipe",
                nodes=[
                    AdmittanceCartesianPosition(
                        name=f"{segment_name}/wipe{waypoint_index}",
                        root_link=self.root_link,
                        tip_link=self.tip_link,
                        goal_point=waypoint,
                        ft_node=self.force_torque_node,
                        desired_force=self.desired_force,
                        damping=_DAMPING,
                        reference_velocity=_WIPE_VELOCITY,
                        threshold=_WIPE_THRESHOLD,
                        weight=_WEIGHT,
                    )
                    for waypoint_index, waypoint in enumerate(waypoints)
                ],
            )
        )
        nodes.append(
            CartesianPosition(
                name=f"{segment_name}/retract",
                root_link=self.root_link,
                tip_link=self.tip_link,
                goal_point=Point3(
                    x=last.x, y=last.y, z=last.z + _APPROACH_HEIGHT,
                    reference_frame=reference_frame,
                ),
                reference_velocity=_APPROACH_VELOCITY,
                weight=_WEIGHT,
            )
        )
        return nodes
