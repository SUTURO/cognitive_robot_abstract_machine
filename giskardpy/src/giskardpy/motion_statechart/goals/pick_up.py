import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Tuple
from typing_extensions import Optional, List

from giskardpy.data_types.exceptions import ForceTorqueSaysNoException
from giskardpy.motion_statechart.context import BuildContext
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
from giskardpy.motion_statechart.ros2_nodes.gripper_control import OpenHand, CloseHand
from giskardpy.motion_statechart.tasks.align_planes import AlignPlanes
from giskardpy.motion_statechart.tasks.cartesian_tasks import (
    CartesianPosition,
    CartesianOrientation,
)
from giskardpy.motion_statechart.tasks.pointing import Pointing
from giskardpy.motion_statechart.test_nodes.test_nodes import ConstTrueNode
from krrood.symbolic_math.symbolic_math import (
    trinary_logic_not,
    trinary_logic_and,
    Scalar,
)
from semantic_digital_twin.robots.abstract_robot import ParallelGripper
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


logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

PICKUP_PREPOSE_DISTANCE = 0.2
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
    simulated_execution: bool = field(default=True, kw_only=True)

    def expand(self, context: BuildContext) -> None:
        super().expand(context)
        logger.debug(f"Object pose: {self.object_geometry.global_pose.to_np()}")
        self.sequence = Sequence(
            [
                OpenHand(simulated_execution=self.simulated_execution),
                GraspingSequence(
                    manipulator=self.manipulator,
                    object_geometry=self.object_geometry,
                    gripper_width=HSR_GRIPPER_WIDTH,
                    gripper_vertical=self.gripper_vertical,
                ),
                CloseHand(ft=self.ft, simulated_execution=self.simulated_execution),
                # PullUp(
                #     manipulator=self.manipulator,
                #     object_geometry=self.object_geometry,
                #     ft=self.ft,
                # ),
            ]
        )
        self.add_node(self.sequence)

    def build(self, context: BuildContext) -> NodeArtifacts:
        artifacts = super().build(context)
        artifacts.observation = self.sequence.observation_variable
        return artifacts


@dataclass(repr=False, eq=False)
class GraspMagic(ABC):
    manipulator: ParallelGripper = field(kw_only=True)
    object_geometry: Body = field(kw_only=True)
    gripper_width: float = field(kw_only=True)
    prefer_front_grasp: bool = field(default=False, kw_only=True)
    gripper_vertical: Optional[bool] = field(default=True, kw_only=True)

    @abstractmethod
    def get_grasp_sequence(
        self, context: BuildContext
    ) -> Tuple[Tuple[CartesianPosition, "Parallel"], Tuple[CartesianPosition, "Parallel"]]:
        """
        Compute all grasp geometry once and return nodes for both phases:
        ((pre_grasp_cart, pre_grasp_align), (grasp_cart, grasp_align))
        """
        pass

    def _select_optimal_grasp_axis(
            self, context: BuildContext, obj_bbox: BoundingBox, obj_to_robot: Vector3
    ) -> Vector3:
        """
        Select the best grasp axis based on gripper width constraints and approach direction.
        Returns the axis most aligned with the robot approach direction that satisfies width constraints.
        Uses transforms to properly handle arbitrary object orientations.
        """
        world_z = Vector3.Z(context.world.root)

        # Define candidate faces: each face is defined by its approach axis
        # For a box, approaching along X means grasping across Y-Z plane
        # The dimensions available for grasping depend on gripper orientation
        candidate_faces = [
            (Vector3.X(self.object_geometry), 'x'),  # Approach along X
            (Vector3.Y(self.object_geometry), 'y'),  # Approach along Y
            (Vector3.Z(self.object_geometry), 'z'),  # Approach along Z
        ]

        valid_faces = []

        for local_axis, axis_name in candidate_faces:
            # Transform the approach axis to world frame to check its orientation
            axis_in_world = context.world.transform(
                spatial_object=local_axis, target_frame=context.world.root
            )
            axis_in_world.reference_frame = context.world.root

            # Determine if this approach axis is vertical or horizontal in world frame
            dot_with_world_z = abs(axis_in_world.dot(world_z))

            # Get the perpendicular dimensions in object frame
            # When approaching along one axis, we grasp across the other two dimensions
            if axis_name == 'x':
                # Approaching along X: grasp across Y-Z plane
                perp_dim1, perp_dim2 = (
                    obj_bbox.width,
                    obj_bbox.height,
                )  # Y and Z dimensions
                perp_axis1 = Vector3.Y(self.object_geometry)
                perp_axis2 = Vector3.Z(self.object_geometry)
            elif axis_name == 'y':
                # Approaching along Y: grasp across X-Z plane
                perp_dim1, perp_dim2 = (
                    obj_bbox.depth,
                    obj_bbox.height,
                )  # X and Z dimensions
                perp_axis1 = Vector3.X(self.object_geometry)
                perp_axis2 = Vector3.Z(self.object_geometry)
            else:  # axis_name == 'z'
                # Approaching along Z: grasp across X-Y plane
                perp_dim1, perp_dim2 = (
                    obj_bbox.depth,
                    obj_bbox.width,
                )  # X and Y dimensions
                perp_axis1 = Vector3.X(self.object_geometry)
                perp_axis2 = Vector3.Y(self.object_geometry)

            # Transform perpendicular axes to world frame
            perp1_world = context.world.transform(
                spatial_object=perp_axis1, target_frame=context.world.root
            )
            perp2_world = context.world.transform(
                spatial_object=perp_axis2, target_frame=context.world.root
            )

            # Determine which perpendicular dimension aligns with the gripper opening
            perp1_dot_z = abs(perp1_world.dot(world_z))
            perp2_dot_z = abs(perp2_world.dot(world_z))

            # Identify which dimension is more vertical and which is more horizontal
            if perp1_dot_z > perp2_dot_z:
                vertical_dim = perp_dim1
                horizontal_dim = perp_dim2
            else:
                vertical_dim = perp_dim2
                horizontal_dim = perp_dim1

            # Check if the object fits based on gripper orientation
            if self.gripper_vertical:
                # Vertical gripper opens horizontally, so horizontal dimension must fit
                graspable_dim = horizontal_dim
            else:
                # Horizontal gripper opens vertically, so vertical dimension must fit
                graspable_dim = vertical_dim

            if graspable_dim <= self.gripper_width:
                valid_faces.append((local_axis, axis_in_world, graspable_dim))

        if not valid_faces:
            raise Exception(
                f"No valid grasp face found. Gripper width: {self.gripper_width}, "
                f"gripper_vertical: {self.gripper_vertical}, "
                f"object dimensions: width={obj_bbox.width:.3f}, depth={obj_bbox.depth:.3f}, height={obj_bbox.height:.3f}"
            )

        # Select the axis most aligned with the approach direction (from robot to object)
        # Use the world-frame axis for the dot product so the comparison is in a consistent frame
        grasp_axis = max(valid_faces, key=lambda x: abs(x[1].dot(obj_to_robot)))[0]
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

        print(f"grasp_axis: {grasp_axis.to_np()}")

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
            threshold: float = 0.01,
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
                threshold=threshold,
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
                    threshold=threshold,
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
                    threshold=threshold,
                    name="enforce_gripper_horizontal",
                )
            )
        else:
            # No orientation constraint (free rotation)
            align_nodes.append(ConstTrueNode())

        return align_nodes


@dataclass(repr=False, eq=False)
class BoxGraspMagic(GraspMagic):
    def get_grasp_sequence(
        self, context: BuildContext
    ) -> Tuple[Tuple[CartesianPosition, Parallel], Tuple[CartesianPosition, Parallel]]:
        # Compute geometry once for both phases
        obj_pose, tool_frame, obj_bbox, obj_to_robot, grasp_axis = self._compute_grasp_geometry(context)

        pre_grasp_point = self._compute_position_along_axis(
            context, obj_pose, obj_bbox, grasp_axis, obj_to_robot, PICKUP_PREPOSE_DISTANCE
        )
        logger.debug(f"Pre grasp point: {pre_grasp_point.to_np()}")
        pre_cart = CartesianPosition(
            root_link=context.world.root,
            tip_link=tool_frame,
            goal_point=pre_grasp_point,
            name="pre_grasp_position",
        )
        # Looser threshold for pre-grasp; the grasp phase re-applies tighter constraints.
        pre_align = Parallel(self._get_orientation_nodes(context, tool_frame, obj_pose, threshold=0.05))

        grasp_point = self._compute_position_along_axis(
            context, obj_pose, obj_bbox, grasp_axis, obj_to_robot, additional_offset=-0.05
        )
        logger.debug(f"Grasp point: {grasp_point.to_np()}")
        grasp_cart = CartesianPosition(
            root_link=context.world.root,
            tip_link=tool_frame,
            goal_point=grasp_point,
            name="grasp_position",
        )
        grasp_align = Parallel(self._get_orientation_nodes(context, tool_frame, obj_pose))

        return (pre_cart, pre_align), (grasp_cart, grasp_align)


class CylinderGraspMagic(GraspMagic):
    def get_grasp_sequence(self, context: BuildContext):
        # TODO: Implement cylinder-specific grasp logic
        pass


@dataclass(repr=False, eq=False)
class _GraspPhase(Goal):
    """
    Internal helper: runs CartesianPosition and orientation in parallel.
    If wait_for_align is True (pre-grasp), completes when both position AND orientation are satisfied.
    If wait_for_align is False (grasp), completes when position only is satisfied.
    """

    _cart: CartesianPosition = field(kw_only=True)
    _align: Parallel = field(kw_only=True)
    _wait_for_align: bool = field(kw_only=True, default=True)

    def expand(self, context: BuildContext) -> None:
        self.add_node(self._cart)
        self.add_node(self._align)

    def build(self, context: BuildContext) -> NodeArtifacts:
        artifacts = super().build(context)
        if self._wait_for_align:
            artifacts.observation = trinary_logic_and(
                self._cart.observation_variable, self._align.observation_variable
            )
        else:
            artifacts.observation = self._cart.observation_variable
        return artifacts


@dataclass(repr=False, eq=False)
class GraspingSequence(Goal):
    """
    Runs the full grasp sequence (pre-grasp then grasp) as a single Goal.
    Computes all grasp geometry once and builds both phases from the result.
    """

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

        (pre_cart, pre_align), (grasp_cart, grasp_align) = self.grasp_magic.get_grasp_sequence(context)

        self._seq = Sequence([
            _GraspPhase(_cart=pre_cart, _align=pre_align, _wait_for_align=True),
            _GraspPhase(_cart=grasp_cart, _align=grasp_align, _wait_for_align=False),
        ])
        self.add_node(self._seq)

    def build(self, context: BuildContext) -> NodeArtifacts:
        artifacts = super().build(context)
        artifacts.observation = self._seq.observation_variable
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

        point = self.object_geometry.global_pose.to_position() + Vector3(
            0, 0, 0.2, reference_frame=context.world.root
        )
        self._cart_position = CartesianPosition(
            root_link=context.world.root,
            tip_link=self.manipulator.tool_frame,
            goal_point=point,
        )
        self._keep_orientation = CartesianOrientation(
            root_link=context.world.root,
            tip_link=self.object_geometry,  # self.manipulator.tool_frame,
            goal_orientation=self.object_geometry.global_pose.to_rotation_matrix(),
            # self.manipulator.tool_frame.global_pose.to_rotation_matrix()
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
