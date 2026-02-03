from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import Tuple

from typing_extensions import Optional, List

from giskardpy.data_types.exceptions import ForceTorqueSaysNoException
from giskardpy.motion_statechart.binding_policy import GoalBindingPolicy
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
    CartesianPosition, CartesianOrientation,
)
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList, JointState
from giskardpy.motion_statechart.tasks.pointing import Pointing
from giskardpy.motion_statechart.test_nodes.test_nodes import ConstTrueNode
from krrood.symbolic_math.symbolic_math import (
    trinary_logic_not,
    trinary_logic_and,
    Scalar,
)
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.abstract_robot import Manipulator, ParallelGripper
from semantic_digital_twin.spatial_types import (
    Vector3,
    Point3,
    HomogeneousTransformationMatrix,
)
from semantic_digital_twin.world_description.geometry import BoundingBox
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
AXIS_ALIGNMENT_THRESHOLD = 0.9  # Threshold for determining primary axis alignment


@dataclass(repr=False, eq=False)
class PickUp(Goal):
    manipulator: ParallelGripper = field(kw_only=True)
    object_geometry: Body = field(kw_only=True)
    ft: bool = field(kw_only=True, default=False)
    gripper_vertical: Optional[bool] = field(default=True, kw_only=True)

    def expand(self, context: BuildContext) -> None:
        super().expand(context)
        grasp_magic = BoxGraspMagic(
            manipulator=self.manipulator,
            object_geometry=self.object_geometry,
            gripper_vertical=self.gripper_vertical,
            gripper_width=HSR_GRIPPER_WIDTH,
        )
        self.sequence = Sequence(
            [
                OpenHand(manipulator=self.manipulator),
                PreGraspPose(
                    manipulator=self.manipulator,
                    object_geometry=self.object_geometry,
                    gripper_width=HSR_GRIPPER_WIDTH,
                    grasp_magic=grasp_magic,
                ),
                Grasping(
                    manipulator=self.manipulator,
                    object_geometry=self.object_geometry,
                    gripper_width=HSR_GRIPPER_WIDTH,
                    grasp_magic=grasp_magic,
                ),
                CloseHand(manipulator=self.manipulator, ft=self.ft),
                # PullUp(manipulator=self.manipulator, object_geometry=self.object_geometry, ft=self.ft),
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
    manipulator: ParallelGripper = field(kw_only=True)

    def expand(self, context: BuildContext) -> None:
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
    manipulator: ParallelGripper = field(kw_only=True)
    object_geometry: Body = field(kw_only=True)
    gripper_width: float = field(kw_only=True)
    prefer_front_grasp: bool = field(default=False, kw_only=True)
    gripper_vertical: Optional[bool] = field(default=True, kw_only=True)

    @abstractmethod
    def get_pre_grasp_nodes(self, context: BuildContext) -> List[MotionStatechartNode]:
        """
        Given an object geometry, returns a list of motion statechart nodes for pre-grasp positioning.
        """
        pass

    @abstractmethod
    def get_grasp_nodes(self, context: BuildContext) -> List[MotionStatechartNode]:
        """
        Given an object geometry, returns a list of motion statechart nodes that grasp the object.
        """
        pass

    def _select_optimal_grasp_axis(
        self, context: BuildContext, obj_bbox: BoundingBox, obj_to_robot: Vector3
    ) -> Vector3:
        """
        Select the best grasp axis based on gripper width constraints and approach direction.
        Returns the axis most aligned with the robot approach direction that satisfies width constraints.
        """
        world_z = Vector3.Z(context.world.root)
        is_obj_vertical = (
            abs(Vector3.Z(self.object_geometry).dot(world_z)) > VERTICAL_DOT_THRESH
        )

        # Define candidate faces: (approach_axis, (face_width, face_height))
        # Width is the dimension perpendicular to approach, height is vertical
        candidate_faces = [
            (Vector3.X(self.object_geometry), (obj_bbox.width, obj_bbox.height)),
            (Vector3.Y(self.object_geometry), (obj_bbox.depth, obj_bbox.height)),
            (Vector3.Z(self.object_geometry), (obj_bbox.depth, obj_bbox.width)),
        ]

        valid_faces = []
        for axis, (face_width, face_height) in candidate_faces:
            # Determine which dimension the gripper must fit:
            # - Vertical gripper grasps horizontal dimension (width)
            # - Horizontal gripper grasps vertical dimension (height)
            if self.gripper_vertical:
                graspable_dim = face_width if is_obj_vertical else face_height
            else:
                graspable_dim = face_height if is_obj_vertical else face_width

            if graspable_dim <= self.gripper_width:
                valid_faces.append((axis, (face_width, face_height)))

        if not valid_faces:
            raise Exception(
                "No valid grasp face found (no face has appropriate dimension "
                "<= gripper width for current gripper orientation)."
            )

        grasp_axis = max(valid_faces, key=lambda x: abs(x[0].dot(obj_to_robot)))[0]
        grasp_axis.reference_frame = self.object_geometry
        grasp_axis.scale(1)
        return grasp_axis

    def _compute_grasp_geometry(
        self, context: BuildContext
    ) -> Tuple[HomogeneousTransformationMatrix, Body, BoundingBox, Vector3, Vector3]:
        """
        Compute common geometric data needed for both pre-grasp and grasp.
        Returns: (obj_pose, tool_frame, obj_bbox, obj_to_robot, grasp_axis)
        """
        obj_pose = self.object_geometry.global_pose
        tool_frame = self.manipulator.tool_frame

        # Compute object bounding box and approach direction
        obj_bbox = self.object_geometry.collision.as_bounding_box_collection_in_frame(
            self.object_geometry
        ).bounding_box()

        robot_pos = self.manipulator.tool_frame.global_pose
        obj_to_robot = robot_pos.to_position() - obj_pose.to_position()
        obj_to_robot.scale(1)

        grasp_axis = self._select_optimal_grasp_axis(context, obj_bbox, obj_to_robot)

        return obj_pose, tool_frame, obj_bbox, obj_to_robot, grasp_axis

    def _compute_offset_along_axis(
        self, obj_bbox: BoundingBox, grasp_axis: Vector3, additional_offset: float = 0.0
    ) -> float:
        """
        Calculate offset distance based on object extent along grasp axis.
        """
        if abs(grasp_axis.x) > AXIS_ALIGNMENT_THRESHOLD:
            half_extent = obj_bbox.depth / 2.0
        elif abs(grasp_axis.y) > AXIS_ALIGNMENT_THRESHOLD:
            half_extent = obj_bbox.width / 2.0
        else:
            half_extent = obj_bbox.height / 2.0

        return half_extent + additional_offset

    def _compute_position_along_axis(
        self,
        context: BuildContext,
        obj_pose: HomogeneousTransformationMatrix,
        obj_bbox: BoundingBox,
        grasp_axis: Vector3,
        obj_to_robot: Vector3,
        additional_offset: float = 0.0,
    ) -> Point3:
        """
        Compute position by offsetting from object center along grasp axis.
        Offset distance accounts for object extent and additional clearance.
        """
        grasp_axis_world = context.world.transform(
            spatial_object=grasp_axis, target_frame=context.world.root
        )
        grasp_axis_world.reference_frame = context.world.root

        dot_along: Scalar = grasp_axis_world.dot(obj_to_robot)
        approach_sign = 1.0 if dot_along >= 0.0 else -1.0

        offset_distance = self._compute_offset_along_axis(
            obj_bbox, grasp_axis, additional_offset
        )
        offset_vector = grasp_axis_world * (offset_distance * approach_sign)

        position = obj_pose.to_position() + offset_vector
        position.reference_frame = context.world.root
        return position

    def _get_orientation_nodes(
        self,
        context: BuildContext,
        tool_frame: KinematicStructureEntity,
        obj_pose: HomogeneousTransformationMatrix,
    ) -> List[MotionStatechartNode]:
        """
        Create orientation constraint nodes based on gripper_vertical parameter.
        """
        align_nodes: List[MotionStatechartNode] = [
            Pointing(
                tip_link=tool_frame,
                goal_point=obj_pose.to_position(),
                root_link=context.world.root,
                pointing_axis=Vector3.Z(tool_frame),
                name="point_at_object",
            )
        ]

        if self.gripper_vertical:
            # Vertical gripper: align gripper X axis to world Z
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
            # Horizontal gripper: align gripper Y axis to world Z
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
            # No orientation constraint (free rotation)
            align_nodes.append(ConstTrueNode())

        return align_nodes


@dataclass(repr=False, eq=False)
class BoxGraspMagic(GraspMagic):
    def get_pre_grasp_nodes(self, context: BuildContext) -> List[MotionStatechartNode]:
        obj_pose, tool_frame, obj_bbox, obj_to_robot, grasp_axis = (
            self._compute_grasp_geometry(context)
        )

        pre_grasp_point = self._compute_position_along_axis(
            context,
            obj_pose,
            obj_bbox,
            grasp_axis,
            obj_to_robot,
            PICKUP_PREPOSE_DISTANCE,
        )

        cart_pose = CartesianPosition(
            root_link=context.world.root,
            tip_link=tool_frame,
            goal_point=pre_grasp_point,
            name="pre_grasp_position",
        )

        # Get orientation nodes
        align_nodes = self._get_orientation_nodes(context, tool_frame, obj_pose)
        parallel = Parallel(align_nodes)

        return [cart_pose, parallel]

    def get_grasp_nodes(self, context: BuildContext) -> List[MotionStatechartNode]:
        obj_pose, tool_frame, obj_bbox, obj_to_robot, grasp_axis = (
            self._compute_grasp_geometry(context)
        )

        grasp_point = self._compute_position_along_axis(
            context,
            obj_pose,
            obj_bbox,
            grasp_axis,
            obj_to_robot,
            additional_offset=-0.05,
        )

        cart = CartesianPosition(
            root_link=context.world.root,
            tip_link=tool_frame,
            goal_point=grasp_point,
            name="Grasping",
        )

        return [cart]


class CylinderGraspMagic(GraspMagic):
    def get_pre_grasp_nodes(self, context: BuildContext) -> List[MotionStatechartNode]:
        # TODO: Implement cylinder-specific pre-grasp logic
        pass

    def get_grasp_nodes(self, context: BuildContext) -> List[MotionStatechartNode]:
        # TODO: Implement cylinder-specific grasp logic
        pass


@dataclass(repr=False, eq=False)
class PreGraspPose(Goal):
    manipulator: ParallelGripper = field(kw_only=True)
    object_geometry: Body = field(kw_only=True)
    gripper_width: float = field(kw_only=True)
    gripper_vertical: Optional[bool] = field(default=True, kw_only=True)
    grasp_magic: Optional[GraspMagic] = field(default=None, kw_only=True)

    def expand(self, context: BuildContext) -> None:
        if self.grasp_magic is None:
            self.grasp_magic = BoxGraspMagic(
                manipulator=self.manipulator,
                object_geometry=self.object_geometry,
                gripper_width=self.gripper_width,
                gripper_vertical=self.gripper_vertical,
            )

        nodes = self.grasp_magic.get_pre_grasp_nodes(context)

        for node in nodes:
            self.add_node(node)
            if isinstance(node, CartesianPosition):
                self._cart_pose = node
            elif isinstance(node, Parallel):
                self.parallel = node

    def build(self, context: BuildContext) -> NodeArtifacts:
        artifacts = super().build(context)
        artifacts.observation = trinary_logic_and(
            self.parallel.observation_variable, self._cart_pose.observation_variable
        )
        return artifacts


@dataclass(repr=False, eq=False)
class Grasping(Goal):
    manipulator: ParallelGripper = field(kw_only=True)
    object_geometry: Body = field(kw_only=True)
    gripper_width: float = field(kw_only=True)
    gripper_vertical: Optional[bool] = field(default=True, kw_only=True)
    grasp_magic: Optional[GraspMagic] = field(default=None, kw_only=True)

    def expand(self, context: BuildContext) -> None:
        if self.grasp_magic is None:
            self.grasp_magic = BoxGraspMagic(
                manipulator=self.manipulator,
                object_geometry=self.object_geometry,
                gripper_width=self.gripper_width,
                gripper_vertical=self.gripper_vertical,
            )

        nodes = self.grasp_magic.get_grasp_nodes(context)

        for node in nodes:
            self.add_node(node)
            if isinstance(node, CartesianPosition):
                self.cart = node

    def build(self, context: BuildContext) -> NodeArtifacts:
        artifacts = super().build(context)
        artifacts.observation = self.cart.observation_variable
        return artifacts


@dataclass(repr=False, eq=False)
class CloseHand(Goal):
    # NOTE: Only works in simulation, giskard cant move gripper irl
    manipulator: ParallelGripper = field(kw_only=True)
    ft: bool = field(kw_only=True, default=False)

    def expand(self, context: BuildContext) -> None:
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
    manipulator: ParallelGripper = field(kw_only=True)
    object_geometry: Body = field(kw_only=True)
    ft: bool = field(kw_only=True, default=False)

    def expand(self, context: BuildContext) -> None:
        super().expand(context)
        if self.ft:
            self._ft = ForceImpactMonitor(threshold=50, topic_name="ft_irgendwas")
            self._cm = CancelMotion(exception=ForceTorqueSaysNoException("No"))
            self._cm.start_condition = trinary_logic_not(self._ft.observation_variable)
            self.add_node(self._ft)
            self.add_node(self._cm)

        point = self.object_geometry.global_pose.to_position() + Vector3(0, 0, 0.2, reference_frame=context.world.root)
        self._cart_position = CartesianPosition(
            root_link=context.world.root,
            tip_link=self.manipulator.tool_frame,
            goal_point=point,
        )
        self._keep_orientation = CartesianOrientation(
            root_link=context.world.root,
            tip_link=self.object_geometry,
            goal_orientation=self.object_geometry.global_pose.to_rotation_matrix()
        )
        self.add_node(self._cart_position)
        self.add_node(self._keep_orientation)

    def build(self, context: BuildContext) -> NodeArtifacts:
        artifacts = super().build(context)

        if self.ft:
            artifacts.observation = trinary_logic_and(
                self._ft.observation_variable, self._cart_position.observation_variable
            )
        else:
            artifacts.observation = self._cart_position.observation_variable
        return artifacts
