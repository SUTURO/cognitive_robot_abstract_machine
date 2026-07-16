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
from giskardpy.motion_statechart.goals.templates import Sequence
from giskardpy.motion_statechart.graph_node import (
    MotionStatechartNode,
    NodeArtifacts,
    Task,
)
from giskardpy.motion_statechart.ros2_nodes.force_torque_monitor import (
    ForceTorqueSymbolNode,
)
from giskardpy.motion_statechart.tasks.admittance_tasks import (
    AdmittanceCartesianPosition,
    LowerUntilContact,
)
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPosition


WipeSegment = list[Point3]
"""The waypoints of one uninterrupted stroke run; the tool lifts between segments."""


@dataclass(eq=False, repr=False)
class AlignTipAxis(Task):
    """Keep an axis of ``tip_link`` pointing along a fixed direction in the
    root frame, e.g. a tool's approach axis pointing at the surface while
    position tasks move it."""

    root_link: KinematicStructureEntity = field(kw_only=True)
    """Root of the kinematic chain. Both vectors are expressed in this frame."""

    tip_link: KinematicStructureEntity = field(kw_only=True)
    """Link whose ``tip_axis`` is aligned."""

    tip_axis: Vector3 = field(kw_only=True)
    """Axis in the tip frame to align, e.g. the gripper's approach axis."""

    root_axis_goal: Vector3 = field(kw_only=True)
    """Direction in the root frame ``tip_axis`` should point along."""

    reference_velocity: float = field(default=0.025, kw_only=True)
    """Angular reference velocity in rad/s."""

    threshold: float = field(default=0.01, kw_only=True)
    """Tilt error in rad below which the alignment counts as achieved."""

    def build(self, context: MotionStatechartContext) -> NodeArtifacts:
        root_T_tip = context.world.compose_forward_kinematics_expression(
            self.root_link, self.tip_link
        )
        root_V_tip_axis = root_T_tip @ self.tip_axis
        tilt_error = root_V_tip_axis.angle_between(self.root_axis_goal)

        artifacts = NodeArtifacts()
        artifacts.constraints.add_vector_goal_constraints(
            frame_V_current=root_V_tip_axis,
            frame_V_goal=self.root_axis_goal,
            reference_velocity=self.reference_velocity,
            quadratic_weight=self.weight,
            name=str(self.name),
        )
        artifacts.observation = tilt_error <= self.threshold
        return artifacts


@dataclass(eq=False, repr=False)
class WipeGoal(Sequence):
    """Table-wiping goal. Each segment expands to ``CartesianPosition(approach)
    -> LowerUntilContact -> AdmittanceCartesianPosition per waypoint ->
    CartesianPosition(retract)``; the segments run as one sequence and the goal
    ends when the last retract completes. Reachability planning is not done
    here; pass segments whose waypoints are reachable from the current base."""

    segments: list[WipeSegment] = field(kw_only=True)
    """Ordered waypoint runs to wipe; the tool lifts between runs."""

    tip_link: KinematicStructureEntity = field(kw_only=True)
    """End link that follows the waypoints (e.g. the gripper tool frame)."""

    root_link: KinematicStructureEntity = field(kw_only=True)
    """Root of the kinematic chain. Usually ``world.root``."""

    force_torque_node: ForceTorqueSymbolNode = field(kw_only=True)
    """Wrench source shared by ``LowerUntilContact`` and the admittance tasks.
    Owned and added to the chart by this goal."""

    parallel_nodes: list[MotionStatechartNode] = field(
        default_factory=list, kw_only=True
    )
    """Extra nodes running in parallel with the strokes (e.g. collision
    avoidance), ended when the strokes finish."""

    approach_collision_rules: list[CollisionRule] = field(
        default_factory=list, kw_only=True
    )
    """Collision rules applied at the start of every segment, e.g. the gripper
    avoids the surface so it descends from above. Empty = matrix unchanged."""

    contact_collision_rules: list[CollisionRule] = field(
        default_factory=list, kw_only=True
    )
    """Collision rules applied before every contact-seeking descent, e.g.
    suspend gripper-vs-surface avoidance. Empty = matrix unchanged."""

    approach_height: float = field(default=0.10, kw_only=True)
    """Vertical offset above the first/last waypoint for approach and retract, in m."""

    contact_force_threshold: float = field(default=3.0, kw_only=True)
    """Force magnitude in N at which ``LowerUntilContact`` declares contact."""

    lower_overshoot: float = field(default=0.05, kw_only=True)
    """How far below the first waypoint Z the descent targets, in m; the force
    observation halts it, so the surface height need not be exact."""

    approach_reference_velocity: float = field(default=0.10, kw_only=True)
    """Reference velocity for the approach/retract motions, in m/s."""

    lower_reference_velocity: float = field(default=0.03, kw_only=True)
    """Reference velocity for the contact-seeking descent, in m/s."""

    wipe_reference_velocity: float = field(default=0.10, kw_only=True)
    """Reference velocity for each wipe waypoint, in m/s."""

    wipe_threshold: float = field(default=0.03, kw_only=True)
    """Distance in m at which a wipe waypoint counts as reached. Looser than
    the task default: under contact, corners may be marginally out of reach."""

    desired_force: Vector3 | None = field(default=None, kw_only=True)
    """Contact force the admittance balances, forwarded to every wipe task."""

    mass: Vector3 | None = field(default=None, kw_only=True)
    """Admittance virtual mass, forwarded to every wipe task."""

    damping: Vector3 = field(
        default_factory=lambda: Vector3(x=20.0, y=20.0, z=100.0), kw_only=True
    )
    """Admittance damping, forwarded to every wipe task. The z default is
    near-critical against a firm surface (~2 kN/m contact with unit virtual
    mass), so the press settles instead of bouncing off; x/y stay soft so the
    strokes yield sideways."""

    stiffness: Vector3 | None = field(default=None, kw_only=True)
    """Admittance stiffness, forwarded to every wipe task."""

    keep_tip_axis_aligned: bool = field(default=False, kw_only=True)
    """Hold the tip link's +Z axis pointing straight down for the whole wipe,
    so the tool faces the surface instead of leaving orientation to the IK."""

    weight: float = DefaultWeights.WEIGHT_ABOVE_CA
    """Quadratic weight of all wipe constraints."""

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

        # Children run in parallel (no super().expand() chaining); only the
        # strokes are sequential.
        self.add_node(self.force_torque_node)
        self._strokes = Sequence(name=f"{self.name}/strokes", nodes=motion_nodes)
        self.add_node(self._strokes)

        if self.keep_tip_axis_aligned:
            tilt = AlignTipAxis(
                name=f"{self.name}/align_tip",
                root_link=self.root_link,
                tip_link=self.tip_link,
                tip_axis=Vector3.Z(reference_frame=self.tip_link),
                root_axis_goal=Vector3(x=0, y=0, z=-1, reference_frame=self.root_link),
                # Full wipe weight folds the arm against the surface; slower
                # than ~0.4 rad/s lags a moving stroke and tips the tool.
                weight=0.2 * self.weight,
                reference_velocity=0.4,
            )
            self.add_node(tilt)
            tilt.end_condition = self._strokes.observation_variable

        for node in self.parallel_nodes:
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
        first_waypoint = waypoints[0]
        last_waypoint = waypoints[-1]
        reference_frame = first_waypoint.reference_frame

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
                    x=first_waypoint.x,
                    y=first_waypoint.y,
                    z=first_waypoint.z + self.approach_height,
                    reference_frame=reference_frame,
                ),
                reference_velocity=self.approach_reference_velocity,
                weight=self.weight,
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
                    x=first_waypoint.x,
                    y=first_waypoint.y,
                    z=first_waypoint.z - self.lower_overshoot,
                    reference_frame=reference_frame,
                ),
                ft_node=self.force_torque_node,
                force_threshold=self.contact_force_threshold,
                reference_velocity=self.lower_reference_velocity,
                weight=self.weight,
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
                        mass=self.mass,
                        damping=self.damping,
                        stiffness=self.stiffness,
                        reference_velocity=self.wipe_reference_velocity,
                        threshold=self.wipe_threshold,
                        weight=self.weight,
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
                    x=last_waypoint.x,
                    y=last_waypoint.y,
                    z=last_waypoint.z + self.approach_height,
                    reference_frame=reference_frame,
                ),
                reference_velocity=self.approach_reference_velocity,
                weight=self.weight,
            )
        )
        return nodes
