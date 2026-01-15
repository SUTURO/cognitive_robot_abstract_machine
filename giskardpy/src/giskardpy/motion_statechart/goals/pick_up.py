from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from giskardpy.data_types.exceptions import ForceTorqueSaysNoException
from giskardpy.motion_statechart.context import BuildContext
from giskardpy.motion_statechart.goals.templates import Sequence, Parallel
from giskardpy.motion_statechart.graph_node import Goal, NodeArtifacts, CancelMotion
from giskardpy.motion_statechart.ros2_nodes.force_torque_monitor import (
    ForceImpactMonitor,
)
from giskardpy.motion_statechart.tasks.align_planes import AlignPlanes
from giskardpy.motion_statechart.tasks.cartesian_tasks import (
    CartesianPose,
    CartesianPosition,
)
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList, JointState
from giskardpy.motion_statechart.tasks.pointing import Pointing
from krrood.symbolic_math.symbolic_math import trinary_logic_not, trinary_logic_and
from pycram.datastructures.pose import PoseStamped
from semantic_digital_twin.robots.abstract_robot import Manipulator
from semantic_digital_twin.spatial_types import Vector3, Point3
from semantic_digital_twin.world_description.world_entity import (
    Body,
    KinematicStructureEntity,
)


class HSRGripper(Enum):
    open_gripper = 1.23
    close_gripper = 0


@dataclass(repr=False, eq=False)
class PickUp(Goal):
    # root_link: KinematicStructureEntity = field(kw_only=True)
    manipulator: Manipulator = field(kw_only=True)
    object_geometry: Body = field(kw_only=True)
    ft: bool = field(kw_only=True, default=False)

    def expand(self, context: BuildContext) -> None:
        super().expand(context)
        self.sequence = Sequence(
            [
                OpenHand(manipulator=self.manipulator),
                PreGraspPose(
                    manipulator=self.manipulator, object_geometry=self.object_geometry
                ),
                Grasping(
                    manipulator=self.manipulator, object_geometry=self.object_geometry
                ),
                CloseHand(manipulator=self.manipulator, ft=self.ft),
                # PullUp(manipulator=self.manipulator, ft=self.ft)
                # Retracting(manipulator=self.manipulator, distance=0.15)
            ]
        )
        self.add_node(self.sequence)

    def build(self, context: BuildContext) -> NodeArtifacts:
        artifacts = super().build(context)
        artifacts.observation = self.sequence.observation_variable
        return artifacts


@dataclass(repr=False, eq=False)
class OpenHand(Goal):
    manipulator: Manipulator = field(kw_only=True)

    def expand(self, context: BuildContext) -> None:
        # TODO remove hsr hardcoding
        self.joint_goal = JointPositionList(
            goal_state=JointState.from_str_dict(
                {"hand_motor_joint": HSRGripper.open_gripper.value}, context.world
            )
        )
        self.add_node(self.joint_goal)

    def build(self, context: BuildContext) -> NodeArtifacts:
        artifacts = super().build(context)
        artifacts.observation = self.joint_goal.observation_variable
        return artifacts


@dataclass(repr=False, eq=False)
class PreGraspPose(Goal):
    manipulator: Manipulator = field(kw_only=True)
    object_geometry: Body = field(kw_only=True)

    def expand(self, context: BuildContext) -> None:
        if not self.manipulator or not self.object_geometry:
            raise Exception("Bad input of PrePose Class")

        obj_pose = self.object_geometry.global_pose
        tool_frame = self.manipulator.tool_frame
        front_facing = self.manipulator.front_facing_axis
        front_facing.reference_frame = tool_frame

        print(f"Tool frame name: {tool_frame.name}")
        object_up = Vector3.Z(self.object_geometry)
        object_up.reference_frame = self.object_geometry

        # TODO Check object dimensions & orientation for best grasp direction
        # Calculate pre-grasp position on robot->object vector
        robot_pos = self.manipulator.tool_frame.global_pose
        obj_to_robot = Vector3()
        obj_to_robot.x = robot_pos.x - obj_pose.x
        obj_to_robot.y = robot_pos.y - obj_pose.y
        obj_to_robot.z = robot_pos.z - obj_pose.z
        obj_to_robot.norm()

        pre_grasp_point = Point3()
        pre_grasp_point.x = obj_pose.x + (obj_to_robot.x * 0.3)
        pre_grasp_point.y = obj_pose.y + (obj_to_robot.y * 0.3)
        pre_grasp_point.z = obj_pose.z
        pre_grasp_point.reference_frame = context.world.root

        self._cart_pose = CartesianPosition(
            root_link=context.world.root,
            tip_link=tool_frame,
            goal_point=pre_grasp_point,
        )
        self.add_node(self._cart_pose)

        self.add_node(
            parallel := Parallel(
                [
                    Pointing(
                        root_link=context.world.root,
                        tip_link=tool_frame,
                        pointing_axis=front_facing,
                        goal_point=obj_pose.to_position(),
                        name="pointing",
                    ),
                    AlignPlanes(
                        tip_link=tool_frame,
                        tip_normal=Vector3.X(self.manipulator.tool_frame),
                        root_link=context.world.root,
                        goal_normal=object_up,
                        name="align",
                    ),
                ]
            )
        )
        self.parallel = parallel

    def build(self, context: BuildContext) -> NodeArtifacts:
        artifacts = super().build(context)
        artifacts.observation = trinary_logic_and(
            self.parallel.observation_variable, self._cart_pose.observation_variable
        )
        return artifacts


@dataclass(repr=False, eq=False)
class Grasping(Goal):
    manipulator: Manipulator = field(kw_only=True)
    object_geometry: Body = field(kw_only=True)

    def expand(self, context: BuildContext) -> None:
        # TODO implement better grasping motion
        # HSR can only really do front grasping
        # Bcs of that pycram pickup did it so that the robot is always grasping the object from its
        # front facing axis (which was the x axis, the longest dimension of the object)
        # even though the object could have fit from a longer side
        goal_point = Point3()
        goal_point.x = self.object_geometry.global_pose.x
        goal_point.y = self.object_geometry.global_pose.y
        goal_point.z = self.object_geometry.global_pose.z
        goal_point.reference_frame = context.world.root

        self.add_node(
            cart := CartesianPosition(
                root_link=context.world.root,
                tip_link=self.manipulator.tool_frame,
                goal_point=goal_point,
            )
        )
        self.cart = cart

    def build(self, context: BuildContext) -> NodeArtifacts:
        artifacts = super().build(context)
        artifacts.observation = self.cart.observation_variable
        return artifacts


@dataclass(repr=False, eq=False)
class CloseHand(Goal):
    manipulator: Manipulator = field(kw_only=True)
    ft: bool = field(kw_only=True, default=False)

    def expand(self, context: BuildContext) -> None:
        # TODO remove hsr hardcoding
        self.joint_goal = JointPositionList(
            goal_state=JointState.from_str_dict(
                {"hand_motor_joint": HSRGripper.close_gripper.value}, context.world
            )
        )
        self.add_node(self.joint_goal)

    def build(self, context: BuildContext) -> NodeArtifacts:
        artifacts = super().build(context)
        artifacts.observation = self.joint_goal.observation_variable
        return artifacts


@dataclass(repr=False, eq=False)
class PullUp(Goal):
    manipulator: Manipulator = field(kw_only=True)
    ft: bool = field(kw_only=True, default=False)

    def expand(self, context: BuildContext) -> None:
        super().expand(context)
        if not self.ft:
            self._ft = ForceImpactMonitor(threshold=50, topic_name="ft_irgendwas")
            self._cm = CancelMotion(exception=ForceTorqueSaysNoException("No"))
            self._cm.start_condition = trinary_logic_not(self._ft.observation_variable)
            self.add_node(self._ft)
            self.add_node(self._cm)

        pose = PoseStamped()
        # magic
        self._cart_pose = CartesianPose(
            root_link=context.world.root,
            tip_link=self.manipulator.tool_frame,
            goal_pose=pose,
        )

    def build(self, context: BuildContext) -> NodeArtifacts:
        artifacts = super().build(context)

        artifacts.observation = trinary_logic_and(
            self._ft.observation_variable, self._cart_pose.observation_variable
        )
        return artifacts
