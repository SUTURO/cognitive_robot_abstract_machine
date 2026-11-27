"""
The action that puts down what an arm holds.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from cramph.node import StatechartNode
from coraplex.robot_plans.actions.base import SequentialAction
from cramph.composites import Parallel
from giskardpy.motion_statechart.mixins import HasGoalPoseParameter
from coraplex.robot_plans.mixins import (
    GraspPoseSequence,
    HasArmParameter,
    HasGraspableParameter,
    HasGraspApproach,
    HasPlacingCruiseLinearVelocityParameter,
    HasTransportCruiseLinearVelocityParameter,
    HasMaxReleaseOpeningVelocityParameter,
    HasRetractCruiseLinearVelocityParameter,
    HasVariablePoses,
    MovesToolCenterPoint,
    OperatesGripper,
    ReattachesBranches,
)
from semantic_digital_twin.collision_checking.collision_matrix import CollisionRule
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionForEndEffector,
)
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from krrood.patterns.field_metadata import JSONMetadata
from krrood_spatial_math.spatial_types import Pose

# %% placing a held object


@dataclass(eq=False, repr=False, kw_only=True)
class PlaceAction(
    MovesToolCenterPoint,
    OperatesGripper,
    ReattachesBranches,
    HasVariablePoses,
    SequentialAction,
    HasArmParameter,
    HasGraspableParameter,
    HasGoalPoseParameter,
    HasGraspApproach,
    HasPlacingCruiseLinearVelocityParameter,
    HasTransportCruiseLinearVelocityParameter,
    HasMaxReleaseOpeningVelocityParameter,
    HasRetractCruiseLinearVelocityParameter,
):
    """
    Puts down :attr:`graspable`, which :attr:`arm` holds, so that it ends up at
    :attr:`goal_pose` however it is held.

    The grasp is measured when the place starts, so the place can be planned before the
    object is picked up.
    """

    _tool_goals: GraspPoseSequence | None = field(
        default=None,
        init=False,
        repr=False,
        metadata=JSONMetadata(serialize=True).as_dict(),
    )
    """
    The tool frame goals releasing the object, set to the release of the grasp measured
    when this place starts.

    Sent along with the place, because a receiver does not expand it again.
    """

    def create_temporary_collision_rules(self) -> list[CollisionRule]:
        """
        :return: An allowance for the gripper of :attr:`arm` to touch the surroundings of
            the object, which it is around from setting it down until it has retracted,
            the moment the object is put back under the world included.
        """
        return [AllowCollisionForEndEffector(end_effector=self.arm.end_effector)]

    def create_steps(self) -> list[StatechartNode]:
        reference_frame = self.goal_pose.reference_frame
        self._tool_goals = GraspPoseSequence(
            pre_grasp=self.create_variable_pose(
                name="pre_grasp", reference_frame=reference_frame
            ),
            grasp=self.create_variable_pose(
                name="grasp", reference_frame=reference_frame
            ),
            retreat=self.create_variable_pose(
                name="retreat", reference_frame=reference_frame
            ),
        )
        # A release runs the grasp backwards: down from above the target, then out
        # along the way the grasp was approached.
        return [
            self.tool_center_point_goal(
                target=self._tool_goals.retreat,
                arm=self.arm,
                cruise_linear_velocity=self.transport_cruise_linear_velocity,
            ),
            self.tool_center_point_goal(
                target=self._tool_goals.grasp,
                arm=self.arm,
                cruise_linear_velocity=self.placing_cruise_linear_velocity,
            ),
            Parallel(
                nodes=[
                    self.gripper_goal_type.create_gripper_goal(
                        configuration=self.arm.end_effector.default_configuration(
                            GripperState.OPEN,
                            finger_velocity=self.max_release_opening_velocity,
                        )
                    ),
                    # Keeps the gripper in place while it opens, whatever else
                    # moves the robot meanwhile.
                    self.tool_center_point_goal(
                        target=self._tool_goals.grasp, arm=self.arm
                    ),
                ]
            ),
            *self.reattach(branch_root=self.graspable.root, new_parent=self.world.root),
            # The gripper starts out around the object it has just let go of.
            self.tool_center_point_goal(
                target=self._tool_goals.pre_grasp,
                arm=self.arm,
                cruise_linear_velocity=self.retract_cruise_linear_velocity,
            ),
        ]

    @property
    def variable_poses(self) -> list[Pose]:
        return self._tool_goals.poses

    def placed_object_pose(self) -> Pose:
        """
        :return: The pose :attr:`graspable` is put down at, measured when the place
            starts: :attr:`goal_pose`.
        """
        return self.goal_pose

    def on_start(self) -> None:
        """
        Differs from :meth:`~coraplex.robot_plans.actions.base.Action.on_start` in also
        setting :attr:`_tool_goals` to the release of the grasp :attr:`arm` has on
        :attr:`graspable` now.
        """
        super().on_start()
        end_effector = self.arm.end_effector
        grasp = GraspCandidate(
            self.graspable,
            end_effector.body_T_grasp(
                body=self.graspable.root,
                tool_P_grasp=end_effector.tool_P_grasp_when_closed,
            ),
        )
        place_sequence = self.grasp_pose_sequence(
            reference_T_grasp=grasp.moved_to(self.placed_object_pose()),
            end_effector=end_effector,
            grasp_candidate=grasp,
        )
        for variable_pose, pose in zip(self._tool_goals.poses, place_sequence.poses):
            self.set_variable_pose(variable_pose=variable_pose, value=pose)


# %% placing a held object turned as it is held


@dataclass(eq=False, repr=False, kw_only=True)
class YawKeepingPlaceAction(PlaceAction):
    """
    Puts down :attr:`graspable` at the position of :attr:`goal_pose`, upright as the
    goal pose says, but turned about the vertical axis as it is held when the place
    starts, so the gripper keeps pointing the way it points.
    """

    def placed_object_pose(self) -> Pose:
        """
        Differs from :meth:`PlaceAction.placed_object_pose` in taking the yaw of the
        held object instead of that of :attr:`goal_pose`.
        """
        reference_frame = self.goal_pose.reference_frame
        reference_T_held = self.world.transform(
            self.graspable.root.global_pose, reference_frame
        )
        _, _, held_yaw = reference_T_held.rotation_matrix.rpy
        roll, pitch, _ = self.goal_pose.rotation_matrix.rpy
        return Pose.from_xyz_rpy(
            x=self.goal_pose.x,
            y=self.goal_pose.y,
            z=self.goal_pose.z,
            roll=roll,
            pitch=pitch,
            yaw=held_yaw,
            reference_frame=reference_frame,
        )
