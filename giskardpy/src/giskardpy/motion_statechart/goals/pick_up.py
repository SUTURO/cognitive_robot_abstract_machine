from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from giskardpy.data_types.exceptions import ForceTorqueSaysNoException
from giskardpy.motion_statechart.context import BuildContext, ExecutionContext
from giskardpy.motion_statechart.data_types import DefaultWeights
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
    # NOTE: Pickup should be called split, meaning grabbing first and then separately retracting
    # because after grasping the object it should get attached to the tool frame in semdt
    manipulator: Manipulator = field(kw_only=True)
    object_geometry: Body = field(kw_only=True)
    ft: bool = field(kw_only=True, default=False)

    def expand(self, context: BuildContext) -> None:
        super().expand(context)
        self.sequence = Sequence(
            [
                # OpenHand(manipulator=self.manipulator),
                PreGraspPose(
                    manipulator=self.manipulator, object_geometry=self.object_geometry
                ),
                Grasping(
                    manipulator=self.manipulator, object_geometry=self.object_geometry
                ),
                # CloseHand(manipulator=self.manipulator, ft=self.ft),
                PullUp(manipulator=self.manipulator, ft=self.ft),
                # Retracting(manipulator=self.manipulator)
            ]
        )
        self.add_node(self.sequence)

    def build(self, context: BuildContext) -> NodeArtifacts:
        artifacts = super().build(context)
        artifacts.observation = self.sequence.observation_variable
        return artifacts

    # @dataclass(repr=False, eq=False)
    # class OpenHand(Goal):
    #     # NOTE: Only works in simulation, giskard cant move gripper irl
    #     manipulator: Manipulator = field(kw_only=True)
    #
    #     def expand(self, context: BuildContext) -> None:
    #         # TODO remove hsr hardcoding
    #         position_list = JointPositionList(
    #             goal_state=JointState.from_str_dict(
    #                 {"hand_motor_joint": HSRGripper.open_gripper.value}, context.world
    #             )
    #         )
    #         self.joint_goal = position_list
    #         self.add_node(self.joint_goal)

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
        # HSR can only really do front grasping
        # Bcs of that pycram pickup did it so that the robot is always grasping the object from its
        # front facing axis (which was the x axis, the longest dimension of the object)
        # even though the object could have fit from a longer side
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


#
# @dataclass(repr=False, eq=False)
# class CloseHand(Goal):
#     # NOTE: Only works in simulation, giskard cant move gripper irl
#     manipulator: Manipulator = field(kw_only=True)
#     ft: bool = field(kw_only=True, default=False)
#
#     def expand(self, context: BuildContext) -> None:
#         # TODO remove hsr hardcoding
#         self.joint_goal = JointPositionList(
#             goal_state=JointState.from_str_dict(
#                 {"hand_motor_joint": HSRGripper.close_gripper.value}, context.world
#             )
#         )
#         self.add_node(self.joint_goal)
#
#     def build(self, context: BuildContext) -> NodeArtifacts:
#         artifacts = super().build(context)
#         artifacts.observation = self.joint_goal.observation_variable
#         return artifacts


@dataclass(repr=False, eq=False)
class PullUp(Goal):
    manipulator: Manipulator = field(kw_only=True)
    ft: bool = field(kw_only=True, default=False)

    def expand(self, context: BuildContext) -> None:
        super().expand(context)
        if self.ft:
            self._ft = ForceImpactMonitor(threshold=50, topic_name="ft_irgendwas")
            self._cm = CancelMotion(exception=ForceTorqueSaysNoException("No"))
            self._cm.start_condition = trinary_logic_not(self._ft.observation_variable)
            self.add_node(self._ft)
            self.add_node(self._cm)

        # Note: Assumes x axis of tool frame is pointing up
        # Transforming point from map to tool frame is problematic, as we then need to evaluate the position at runtime
        point = Point3(0.15, 0.0, 0.0, reference_frame=self.manipulator.tool_frame)
        self._cart_position = CartesianPosition(
            root_link=context.world.root,
            tip_link=self.manipulator.tool_frame,
            goal_point=point,
        )
        self.add_node(self._cart_position)

    def build(self, context: BuildContext) -> NodeArtifacts:
        artifacts = super().build(context)

        if self.ft:
            artifacts.observation = trinary_logic_and(
                self._ft.observation_variable, self._cart_position.observation_variable
            )
        else:
            artifacts.observation = self._cart_position.observation_variable
        return artifacts


@dataclass(repr=False, eq=False)
class Retracting(Goal):
    """
    Retracts the tool frame of a manipulator by a certain distance.
    Generic implementation that works with any reference frame.
    """

    manipulator: Manipulator = field(kw_only=True)
    distance: float = field(default=0.15, kw_only=True)
    reference_frame: Optional[KinematicStructureEntity] = field(
        default=None, kw_only=True
    )
    velocity: float = field(default=0.1, kw_only=True)
    weight: float = field(default=DefaultWeights.WEIGHT_ABOVE_CA, kw_only=True)

    def expand(self, context: BuildContext) -> None:
        # Default to manipulator's tool frame if no reference frame is provided
        ref_frame = (
            self.reference_frame
            if self.reference_frame
            else self.manipulator.tool_frame
        )
        tip_link = self.manipulator.tool_frame
        root_link = context.world.root

        # Hand or gripper frames have an outgoing z axis,
        # base frames have an outgoing x axis
        direction = Vector3()
        if "hand" in ref_frame.name.name or "gripper" in ref_frame.name.name:
            direction.z = -self.distance
        else:
            direction.x = -self.distance

        direction.reference_frame = ref_frame

        # TODO: Fix so that it doesnt use transform (makes it so that Point is evaluated not at runtime)
        goal_point = Point3(0, 0, 0, reference_frame=tip_link)
        goal_point = context.world.transform(
            target_frame=ref_frame, spatial_object=goal_point
        )
        goal_point.x += direction.x
        goal_point.y += direction.y
        goal_point.z += direction.z

        world_goal_point = context.world.transform(
            target_frame=root_link, spatial_object=goal_point
        )

        # Linear movement task
        self.cart_pos = CartesianPosition(
            root_link=root_link,
            tip_link=tip_link,
            goal_point=world_goal_point,
            weight=self.weight,
        )

        # Keep Orientation task (Align current tool Z with world Z or keep it fixed)
        self.keep_ori = AlignPlanes(
            root_link=root_link,
            tip_link=tip_link,
            tip_normal=Vector3.Z(tip_link),
            goal_normal=context.world.transform(
                target_frame=root_link, spatial_object=Vector3.Z(tip_link)
            ),
            weight=self.weight,
        )

        self.add_node(parallel := Parallel([self.cart_pos, self.keep_ori]))
        self.parallel = parallel

    def build(self, context: BuildContext) -> NodeArtifacts:
        artifacts = super().build(context)
        # Goal is finished when both position and orientation constraints are satisfied
        artifacts.observation = self.parallel.observation_variable
        return artifacts

    def on_start(self, context: ExecutionContext):
        super().on_start(context)
