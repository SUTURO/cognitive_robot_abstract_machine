from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing_extensions import Optional, List

from giskardpy.data_types.exceptions import ForceTorqueSaysNoException
from giskardpy.motion_statechart.context import BuildContext, ExecutionContext
from giskardpy.motion_statechart.data_types import DefaultWeights
from giskardpy.motion_statechart.goals.templates import Sequence, Parallel
from giskardpy.motion_statechart.graph_node import (
    Goal,
    NodeArtifacts,
    CancelMotion,
    MotionStatechartNode,
)
from giskardpy.motion_statechart.ros2_nodes.force_torque_monitor import (
    ForceImpactMonitor,
)
from giskardpy.motion_statechart.tasks.align_planes import AlignPlanes
from giskardpy.motion_statechart.tasks.cartesian_tasks import (
    CartesianPosition,
)
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList, JointState
from giskardpy.motion_statechart.tasks.pointing import Pointing
from giskardpy.motion_statechart.test_nodes.test_nodes import ConstTrueNode
from krrood.symbolic_math.symbolic_math import (
    trinary_logic_not,
    trinary_logic_and,
    Scalar,
)
from semantic_digital_twin.robots.abstract_robot import Manipulator, ParallelGripper
from semantic_digital_twin.spatial_types import (
    Vector3,
    Point3,
    HomogeneousTransformationMatrix,
)
from semantic_digital_twin.world_description.world_entity import (
    Body,
    KinematicStructureEntity,
)


class HSRGripper(Enum):
    open_gripper = 1.23
    close_gripper = 0


PICKUP_PREPOSE_DISTANCE = 0.03
HSR_GRIPPER_WIDTH = 0.15
PULLUP_HEIGHT = 0.1

VERTICAL_DOT_THRESH = 0.85  # dot with world Z considered vertical
HORIZONTAL_DOT_THRESH = 0.25  # dot with world Z considered horizontal


@dataclass(repr=False, eq=False)
class PickUp(Goal):
    # root_link: KinematicStructureEntity = field(kw_only=True)
    # NOTE: Pickup should be called split, meaning grabbing first and then separately retracting
    # because after grasping the object it should get attached to the tool frame in semdt
    manipulator: Manipulator = field(kw_only=True)  # TODO ParallelGripper instead
    object_geometry: Body = field(kw_only=True)
    ft: bool = field(kw_only=True, default=False)

    def expand(self, context: BuildContext) -> None:
        super().expand(context)
        self.sequence = Sequence(
            [
                OpenHand(manipulator=self.manipulator),
                PreGraspPose(
                    manipulator=self.manipulator,
                    object_geometry=self.object_geometry,
                    gripper_width=HSR_GRIPPER_WIDTH,
                ),
                Grasping(
                    manipulator=self.manipulator, object_geometry=self.object_geometry
                ),
                CloseHand(manipulator=self.manipulator, ft=self.ft),
                PullUp(manipulator=self.manipulator, ft=self.ft),
                # Retracting(manipulator=self.manipulator)
            ]
        )
        self.add_node(self.sequence)

    def build(self, context: BuildContext) -> NodeArtifacts:
        artifacts = super().build(context)
        artifacts.observation = self.sequence.observation_variable
        return artifacts


@dataclass(repr=False, eq=False)
class OpenHand(Goal):
    # NOTE: Only works in simulation, giskard cant move gripper irl
    manipulator: Manipulator = field(kw_only=True)

    def expand(self, context: BuildContext) -> None:
        # TODO remove hsr hardcoding
        position_list = JointPositionList(
            goal_state=JointState.from_str_dict(
                {"hand_motor_joint": HSRGripper.open_gripper.value}, context.world
            )
        )
        self.joint_goal = position_list
        self.add_node(self.joint_goal)

    def build(self, context: BuildContext) -> NodeArtifacts:
        artifacts = super().build(context)
        artifacts.observation = self.joint_goal.observation_variable
        return artifacts


@dataclass(repr=False, eq=False)
class GraspMagic(ABC):
    manipulator: Manipulator = field(kw_only=True)
    object_geometry: Body = field(kw_only=True)
    gripper_width: float = field(kw_only=True)
    prefer_front_grasp: bool = field(default=False, kw_only=True)
    gripper_vertical: Optional[bool] = field(default=True, kw_only=True)

    @abstractmethod
    def get_pre_grasp(self) -> List[MotionStatechartNode]:
        pass

    @abstractmethod
    def get_grasp_nodes(self) -> List[MotionStatechartNode]:
        """
        Given an object geometry, returns a list of motion statechart nodes that grasp the object.
        :return:
        """


@dataclass(repr=False, eq=False)
class BoxGraspMagic(GraspMagic):
    def get_pre_grasp(self) -> List[MotionStatechartNode]:
        pass

    def get_grasp_nodes(self) -> List[MotionStatechartNode]:
        pass


@dataclass(repr=False, eq=False)
class PreGraspPose(Goal):
    manipulator: Manipulator = field(kw_only=True)
    object_geometry: Body = field(kw_only=True)
    gripper_width: float = field(kw_only=True)
    gripper_vertical: Optional[bool] = field(default=True, kw_only=True)

    def expand(self, context: BuildContext) -> None:
        obj_pose = self.object_geometry.global_pose
        tool_frame = self.manipulator.tool_frame
        front_facing = self.manipulator.front_facing_axis
        front_facing.reference_frame = tool_frame

        obj_collision = self.object_geometry.collision
        obj_bbox = obj_collision.as_bounding_box_collection_in_frame(
            self.object_geometry
        ).bounding_box()

        robot_pos = self.manipulator.tool_frame.global_pose
        obj_to_robot = robot_pos.to_position() - obj_pose.to_position()
        obj_to_robot.norm()
        print(f"Object->robot vector on expand: {obj_to_robot}")

        # According to perception we can assume that the z axis points up
        faces = [
            # (axis_vector, (face_dim1, face_dim2))
            (
                Vector3.X(self.object_geometry),
                (obj_bbox.width, obj_bbox.height),
            ),  # approach along X -> face is YxZ
            (
                Vector3.Y(self.object_geometry),
                (obj_bbox.depth, obj_bbox.height),
            ),  # approach along Y -> face is XxZ
            (
                Vector3.Z(self.object_geometry),
                (obj_bbox.depth, obj_bbox.width),
            ),  # approach along Z -> face is XxY
        ]

        valid_faces = [
            (v, (d1, d2)) for v, (d1, d2) in faces if min(d1, d2) <= self.gripper_width
        ]
        if not valid_faces:
            raise Exception(
                "No valid grasp face found (no face has a face-dimension <= gripper width)."
            )
        grasp_axis = max(valid_faces, key=lambda x: abs(x[0].dot(obj_to_robot)))[0]

        grasp_axis.norm()
        print(f"Grasp axis: {grasp_axis.to_np()}")

        dot_along = grasp_axis.dot(obj_to_robot)
        sign = 1.0 if dot_along >= Scalar(0.0) else -1.0

        if abs(grasp_axis.x) > 0.9:
            half_extent = obj_bbox.depth / 2.0
        elif abs(grasp_axis.y) > 0.9:
            half_extent = obj_bbox.width / 2.0
        else:
            half_extent = obj_bbox.height / 2.0

        offset_distance = half_extent + PICKUP_PREPOSE_DISTANCE
        offset_vector = grasp_axis * (offset_distance * sign)

        pre_grasp_point = obj_pose.to_position() + offset_vector
        pre_grasp_point.reference_frame = context.world.root

        self._cart_pose = CartesianPosition(
            root_link=context.world.root,
            tip_link=tool_frame,
            goal_point=pre_grasp_point,
        )
        self.add_node(self._cart_pose)

        align_nodes = []

        align_nodes.append(
            AlignPlanes(
                tip_link=tool_frame,
                tip_normal=Vector3.Z(tool_frame),
                root_link=context.world.root,
                goal_normal=grasp_axis,
                name="align_orientation_to_grasp_axis",
            )
        )

        # If gripper_vertical == True: align gripper X to world Z (jaws vertical)
        # If gripper_vertical == False: align gripper Z to world Z (jaws horizontal)
        # If gripper_vertical is None: no additional constraint (free rotation)
        if self.gripper_vertical:
            align_nodes.append(
                AlignPlanes(
                    tip_link=tool_frame,
                    tip_normal=Vector3.X(tool_frame),
                    root_link=context.world.root,
                    goal_normal=Vector3.Z(context.world.root),
                    name="enforce_gripper_vertical",
                )
            )
        elif self.gripper_vertical is False:
            align_nodes.append(
                AlignPlanes(
                    tip_link=tool_frame,
                    tip_normal=Vector3.Y(tool_frame),
                    root_link=context.world.root,
                    goal_normal=Vector3.Z(context.world.root),
                    name="enforce_gripper_horizontal",
                )
            )
        else:
            align_nodes.append(ConstTrueNode())

        self.add_node(parallel := Parallel(align_nodes))
        self.parallel = parallel

    def build(self, context: BuildContext) -> NodeArtifacts:
        artifacts = super().build(context)
        artifacts.observation = trinary_logic_and(
            self.parallel.observation_variable, self._cart_pose.observation_variable
        )
        return artifacts


# def on_end(self, context: ExecutionContext):
#     super().on_end(context)
#     print(
#         f'On end gripper position: x={self.manipulator.tool_frame.global_pose.x}, y={self.manipulator.tool_frame.global_pose.y}, z={self.manipulator.tool_frame.global_pose.z}')

# def get_grasp_width(self, context: ExecutionContext) -> None:
#     # Evaluates grip at node addition in msc, so goal doesnt know if grip tightened/expanded during msc execution
#     finger_right = context.world.get_kinematic_structure_entity_by_name("hand_r_finger_tip_frame")
#     finger_left = context.world.get_kinematic_structure_entity_by_name("hand_l_finger_tip_frame")
#     grip_width = (finger_left.global_pose.to_position() - finger_right.global_pose.to_position()).norm()
#     print(f"Grip width on expand: {grip_width.to_np()}")


@dataclass(repr=False, eq=False)
class Grasping(Goal):
    manipulator: Manipulator = field(kw_only=True)
    object_geometry: Body = field(kw_only=True)

    def expand(self, context: BuildContext) -> None:
        # TODO implement better grasping motion w feedback
        # TODO rim offset: object_dimension_in_approach_direction / 2
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
    # NOTE: Only works in simulation, giskard cant move gripper irl
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
