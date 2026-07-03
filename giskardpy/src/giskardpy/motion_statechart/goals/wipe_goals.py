from __future__ import division

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from semantic_digital_twin.collision_checking.collision_rules import CollisionRule
from semantic_digital_twin.spatial_types import Point3, Vector3
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.connections import DifferentialDrive
from semantic_digital_twin.world_description.world_entity import (
    KinematicStructureEntity,
)

from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.data_types import DefaultWeights
from giskardpy.motion_statechart.goals.cartesian_goals import DifferentialDriveBaseGoal
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


WipeSegment = Tuple[Optional[Pose], List[Point3]]
"""
One wipe segment: an optional base pose to drive to first, followed by the
waypoints to stroke across with admittance Z. ``base_pose=None`` keeps the
robot at its current base location.
"""


@dataclass(eq=False, repr=False)
class AlignTipAxis(Task):
    """
    Keep an axis of ``tip_link`` pointing along a fixed direction in the root
    frame.

    The :class:`WipeGoal` only constrains the tip *position*, so the IK is free
    to reach a waypoint with the tool in any orientation (often pointing up).
    Running this task in parallel with the wipe holds the tool's approach axis
    pointing at the surface, so the gripper -- and any sponge it carries -- face
    the table for the whole motion.
    """

    root_link: KinematicStructureEntity = field(kw_only=True)
    """Root of the kinematic chain. Both vectors are expressed in this frame."""

    tip_link: KinematicStructureEntity = field(kw_only=True)
    """Link whose ``tip_axis`` is aligned."""

    tip_axis: Vector3 = field(kw_only=True)
    """Axis in the tip frame to align, e.g. the gripper's approach axis."""

    root_axis_goal: Vector3 = field(kw_only=True)
    """Direction in the root frame ``tip_axis`` should point along."""

    reference_velocity: float = field(default=0.025, kw_only=True)
    """Angular reference velocity in rad/s. 0.001 was too small to move the
    real robot, so keep it well above that."""

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
    """
    High-level table-wiping goal.

    Each segment is executed as the sequence
    ``DifferentialDriveBaseGoal? -> CartesianPosition(approach) ->
    LowerUntilContact -> Sequence(AdmittanceCartesianPosition*) ->
    CartesianPosition(retract)``. Segments are concatenated under the outer
    ``Sequence``, so the wipe ends when the last segment's retract completes.

    Reachability planning is **not** done here; pass in a pre-segmented list
    where every base pose makes its waypoints reachable.

    The Z-axis admittance behaviour comes from
    :class:`AdmittanceCartesianPosition`; the per-waypoint admittance
    parameters (``mass``, ``damping``, ``stiffness``, ``desired_force``)
    are forwarded so the caller controls the contact dynamics.
    """

    segments: List[WipeSegment] = field(kw_only=True)
    """Ordered list of ``(base_pose, waypoints)`` to wipe."""

    tip_link: KinematicStructureEntity = field(kw_only=True)
    """End link that follows the waypoints (e.g. the gripper tool frame)."""

    root_link: KinematicStructureEntity = field(kw_only=True)
    """Root of the kinematic chain. Usually ``world.root``."""

    ft_node: ForceTorqueSymbolNode = field(kw_only=True)
    """Force/torque symbol source shared by ``LowerUntilContact`` and the admittance tasks."""

    parallel_nodes: List[MotionStatechartNode] = field(
        default_factory=list, kw_only=True
    )
    """Extra nodes to run in parallel with the wipe strokes (e.g. collision
    avoidance), ending when the strokes finish -- like the tilt constraint."""

    approach_collision_rules: List[CollisionRule] = field(
        default_factory=list, kw_only=True
    )
    """Collision rules applied at the start of every segment (before the
    approach), e.g. the gripper avoids the surface so it descends from above
    instead of driving through it. Empty = leave the matrix unchanged."""

    contact_collision_rules: List[CollisionRule] = field(
        default_factory=list, kw_only=True
    )
    """Collision rules applied before every contact-seeking descent, e.g. suspend
    gripper-vs-surface avoidance (base still avoided) so the tool can press on.
    Empty = leave the matrix unchanged."""

    diff_drive_connection: Optional[DifferentialDrive] = field(
        default=None, kw_only=True
    )
    """Diff drive used to relocate the base. Auto-detected if there is exactly one."""

    approach_height: float = field(default=0.10, kw_only=True)
    """Vertical offset above the first/last waypoint for approach and retract, in m."""

    contact_force_threshold: float = field(default=3.0, kw_only=True)
    """Force magnitude in N at which ``LowerUntilContact`` declares contact."""

    lower_overshoot: float = field(default=0.05, kw_only=True)
    """How far below the first waypoint Z the lowering motion targets, in m. The
    surface depth need not be exact; the force observation halts the descent."""

    approach_reference_velocity: float = field(default=0.10, kw_only=True)
    """Reference velocity for the approach/retract motions, in m/s."""

    lower_reference_velocity: float = field(default=0.03, kw_only=True)
    """Reference velocity for the contact-seeking descent, in m/s."""

    wipe_reference_velocity: float = field(default=0.05, kw_only=True)
    """Reference velocity for each wipe waypoint, in m/s."""

    wipe_threshold: float = field(default=0.03, kw_only=True)
    """Distance at which a wipe waypoint counts as reached, in m. Forwarded to
    every ``AdmittanceCartesianPosition``. Deliberately looser than the task's
    own default: under contact the tool hovers at the surface and corners may
    be marginally out of reach, so a sub-cm tolerance stalls the sequence."""

    desired_force: Optional[Vector3] = field(default=None, kw_only=True)
    """Forwarded to every ``AdmittanceCartesianPosition``."""

    mass: Optional[Vector3] = field(default=None, kw_only=True)
    damping: Optional[Vector3] = field(default=None, kw_only=True)
    stiffness: Optional[Vector3] = field(default=None, kw_only=True)
    inertia_compensation: Optional[Vector3] = field(default=None, kw_only=True)
    acceleration_feedforward_gain: float = field(default=0.0, kw_only=True)

    keep_tip_axis_aligned: bool = field(default=False, kw_only=True)
    """If set, hold the tip link's ``tip_axis`` pointing along ``tip_axis_goal``
    (root frame) for the whole wipe, via a parallel :class:`AlignTipAxis`
    constraint. Keeps the tool -- and any sponge it carries -- pointed at the
    surface instead of letting the position-only tasks leave orientation to the
    IK. Off by default, so existing callers are unchanged."""

    tip_axis: Optional[Vector3] = field(default=None, kw_only=True)
    """Tip-frame axis to align when ``keep_tip_axis_aligned``. Defaults to +Z
    (the HSR gripper's approach axis)."""

    tip_axis_goal: Optional[Vector3] = field(default=None, kw_only=True)
    """Root-frame direction the tip axis should point along. Defaults to -Z of
    ``root_link`` (straight down), so the tool points at the table."""

    tilt_weight: Optional[float] = field(default=None, kw_only=True)
    """Weight of the alignment constraint. Defaults to the wipe ``weight``."""

    tilt_reference_velocity: float = field(default=0.2, kw_only=True)
    """Angular correction speed of the alignment, in rad/s. Must be fast enough
    to re-point the tool while it moves; the slow default for a real robot
    (0.025) lags far behind a moving wipe and leaves the tool nearly flat."""

    weight: float = DefaultWeights.WEIGHT_ABOVE_CA

    _strokes: Optional[Sequence] = field(default=None, init=False, repr=False)
    """The inner sequence of approach/lower/wipe/retract tasks. The goal's
    observation tracks this, so the motion ends when the strokes finish, not
    when the parallel tilt constraint happens to be satisfied."""

    def expand(self, context: MotionStatechartContext) -> None:
        if not self.segments:
            return

        motion_nodes: List[MotionStatechartNode] = []
        for segment_index, (base_pose, waypoints) in enumerate(self.segments):
            if not waypoints:
                continue
            motion_nodes.extend(
                self._segment_nodes(segment_index, base_pose, waypoints)
            )
        if not motion_nodes:
            return

        self.add_node(self.ft_node)

        # The strokes run as a sequence; the tilt (if any) runs in parallel,
        # active until the strokes finish. Not calling super().expand() keeps
        # the two siblings parallel instead of chaining them.
        self._strokes = Sequence(name=f"{self.name}/strokes", nodes=motion_nodes)
        self.add_node(self._strokes)

        if self.keep_tip_axis_aligned:
            tilt = AlignTipAxis(
                name=f"{self.name}/align_tip",
                root_link=self.root_link,
                tip_link=self.tip_link,
                tip_axis=(
                    self.tip_axis
                    if self.tip_axis is not None
                    else Vector3.Z(reference_frame=self.tip_link)
                ),
                root_axis_goal=(
                    self.tip_axis_goal
                    if self.tip_axis_goal is not None
                    else Vector3(x=0, y=0, z=-1, reference_frame=self.root_link)
                ),
                reference_velocity=self.tilt_reference_velocity,
                # A soft objective by default: aligning at the full wipe weight
                # fights surface contact and folds the arm, so default to a
                # fraction of it. Callers can override with ``tilt_weight``.
                weight=(
                    self.tilt_weight
                    if self.tilt_weight is not None
                    else 0.2 * self.weight
                ),
            )
            self.add_node(tilt)
            tilt.end_condition = self._strokes.observation_variable

        # Collision avoidance (or anything else the caller wants live during the
        # wipe) runs parallel to the strokes and stops when they finish.
        for node in self.parallel_nodes:
            self.add_node(node)
            node.end_condition = self._strokes.observation_variable

    def build(self, context: MotionStatechartContext) -> NodeArtifacts:
        if self._strokes is None:
            return NodeArtifacts()
        return NodeArtifacts(observation=self._strokes.observation_variable)

    def _segment_nodes(
        self,
        segment_index: int,
        base_pose: Optional[Pose],
        waypoints: List[Point3],
    ) -> List[MotionStatechartNode]:
        segment_name = f"{self.name}/segment{segment_index}"
        first_waypoint = waypoints[0]
        last_waypoint = waypoints[-1]
        reference_frame = first_waypoint.reference_frame

        nodes: List[MotionStatechartNode] = []

        # Approach phase: gripper avoids the surface (comes down from above).
        if self.approach_collision_rules:
            nodes.append(
                UpdateTemporaryCollisionRules(
                    name=f"{segment_name}/approach_collision",
                    temporary_rules=self.approach_collision_rules,
                )
            )

        if base_pose is not None:
            nodes.append(
                DifferentialDriveBaseGoal(
                    name=f"{segment_name}/drive",
                    diff_drive_connection=self.diff_drive_connection,
                    goal_pose=base_pose,
                    weight=self.weight,
                )
            )

        approach_point = Point3(
            x=first_waypoint.x,
            y=first_waypoint.y,
            z=first_waypoint.z + self.approach_height,
            reference_frame=reference_frame,
        )
        nodes.append(
            CartesianPosition(
                name=f"{segment_name}/approach",
                root_link=self.root_link,
                tip_link=self.tip_link,
                goal_point=approach_point,
                reference_velocity=self.approach_reference_velocity,
                weight=self.weight,
            )
        )

        # Contact phase: suspend gripper-vs-surface avoidance so it can press on
        # (the base and the rest of the robot stay avoided).
        if self.contact_collision_rules:
            nodes.append(
                UpdateTemporaryCollisionRules(
                    name=f"{segment_name}/contact_collision",
                    temporary_rules=self.contact_collision_rules,
                )
            )

        descend_point = Point3(
            x=first_waypoint.x,
            y=first_waypoint.y,
            z=first_waypoint.z - self.lower_overshoot,
            reference_frame=reference_frame,
        )
        nodes.append(
            LowerUntilContact(
                name=f"{segment_name}/lower",
                root_link=self.root_link,
                tip_link=self.tip_link,
                goal_point=descend_point,
                ft_node=self.ft_node,
                force_threshold=self.contact_force_threshold,
                reference_velocity=self.lower_reference_velocity,
                weight=self.weight,
            )
        )

        wipe_tasks = [
            AdmittanceCartesianPosition(
                name=f"{segment_name}/wipe{waypoint_index}",
                root_link=self.root_link,
                tip_link=self.tip_link,
                goal_point=waypoint,
                ft_node=self.ft_node,
                desired_force=self.desired_force,
                mass=self.mass,
                damping=self.damping,
                stiffness=self.stiffness,
                inertia_compensation=self.inertia_compensation,
                acceleration_feedforward_gain=self.acceleration_feedforward_gain,
                reference_velocity=self.wipe_reference_velocity,
                threshold=self.wipe_threshold,
                weight=self.weight,
            )
            for waypoint_index, waypoint in enumerate(waypoints)
        ]
        nodes.append(Sequence(name=f"{segment_name}/wipe", nodes=wipe_tasks))

        retract_point = Point3(
            x=last_waypoint.x,
            y=last_waypoint.y,
            z=last_waypoint.z + self.approach_height,
            reference_frame=reference_frame,
        )
        nodes.append(
            CartesianPosition(
                name=f"{segment_name}/retract",
                root_link=self.root_link,
                tip_link=self.tip_link,
                goal_point=retract_point,
                reference_velocity=self.approach_reference_velocity,
                weight=self.weight,
            )
        )
        return nodes
