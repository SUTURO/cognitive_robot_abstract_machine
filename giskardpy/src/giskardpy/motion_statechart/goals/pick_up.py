from dataclasses import dataclass, field
from enum import Enum

from giskardpy.motion_statechart.context import BuildContext
from giskardpy.motion_statechart.goals.templates import Sequence
from giskardpy.motion_statechart.graph_node import Goal, NodeArtifacts
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList, JointState
from pycram.datastructures.pose import PoseStamped
from semantic_digital_twin.world_description.world_entity import (
    KinematicStructureEntity,
    Body,
)


class HSRGripper(Enum):
    open_gripper = 1.23
    close_gripper = 0


@dataclass(repr=False, eq=False)
class PickUp(Goal):
    root_link: KinematicStructureEntity = field(kw_only=True)
    tip_link: KinematicStructureEntity = field(kw_only=True)
    object_geometry: Body = field(kw_only=True)
    ft: bool = field(kw_only=True, default=False)

    def expand(self, context: BuildContext) -> None:
        super().expand(context)
        Sequence([OpenHand, PrePose, Grasping, CloseHand, PullUp])

    def build(self, context: BuildContext) -> NodeArtifacts:
        return super().build(context)


@dataclass(repr=False, eq=False)
class OpenHand(Goal):
    def expand(self, context: BuildContext) -> None:
        joint_goal = JointPositionList(
            goal_state=JointState.from_str_dict(
                {"hand_motor_joint": HSRGripper.open_gripper.value}, context.world
            )
        )
        self.add_node(joint_goal)

    def build(self, context: BuildContext) -> NodeArtifacts:
        return super().build(context)


@dataclass(repr=False, eq=False)
class PrePose(Goal):
    tip_link: KinematicStructureEntity = field(kw_only=True)

    def expand(self, context: BuildContext) -> None:
        super().expand(context)

        # Orient to front facing axis of object
        pose = PoseStamped()

        pose = CartesianPose(
            root_link=context.world.root, tip_link=self.tip_link, goal_pose=pose
        )

        self.add_node(pose)

    def build(self, context: BuildContext) -> NodeArtifacts:
        return super().build(context)


@dataclass(repr=False, eq=False)
class Grasping(Goal):
    tip_link: KinematicStructureEntity = field(kw_only=True)

    def expand(self, context: BuildContext) -> None:
        super().expand(context)
        # Approach the object with the gripper

    def build(self, context: BuildContext) -> NodeArtifacts:
        return super().build(context)


@dataclass(repr=False, eq=False)
class CloseHand(Goal):
    tip_link: KinematicStructureEntity = field(kw_only=True)
    ft: bool = field(kw_only=True, default=False)

    def expand(self, context: BuildContext) -> None:
        joint_goal = JointPositionList(
            goal_state=JointState.from_str_dict(
                {"hand_motor_joint": HSRGripper.close_gripper.value}, context.world
            )
        )
        self.add_node(joint_goal)

    def build(self, context: BuildContext) -> NodeArtifacts:
        return super().build(context)


@dataclass(repr=False, eq=False)
class PullUp(Goal):
    tip_link: KinematicStructureEntity = field(kw_only=True)
    ft: bool = field(kw_only=True, default=False)

    def expand(self, context: BuildContext) -> None:
        super().expand(context)

    def build(self, context: BuildContext) -> NodeArtifacts:
        return super().build(context)
