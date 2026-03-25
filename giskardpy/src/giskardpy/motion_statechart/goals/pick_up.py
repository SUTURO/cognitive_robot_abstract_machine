import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Tuple
from typing_extensions import Optional, List

from giskardpy.data_types.exceptions import ForceTorqueSaysNoException
from giskardpy.motion_statechart.context import MotionStatechartContext
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
from giskardpy.motion_statechart.data_types import DefaultWeights
from giskardpy.motion_statechart.goals.collision_avoidance import (
    SelfCollisionAvoidance,
    ExternalCollisionAvoidance,
    ExternalCollisionDistanceMonitor,
    UpdateTemporaryCollisionRules,
    make_external_collision_rules,
)
from giskardpy.motion_statechart.tasks.cartesian_tasks import (
    CartesianPosition,
    CartesianOrientation,
)
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionRule,
)
from semantic_digital_twin.world_description.geometry import Cylinder
from krrood.symbolic_math.symbolic_math import trinary_logic_not, trinary_logic_and
from semantic_digital_twin.robots.abstract_robot import ParallelGripper
from semantic_digital_twin.spatial_types import (
    Vector3,
    Point3,
    HomogeneousTransformationMatrix,
    RotationMatrix,
)
from semantic_digital_twin.world_description.geometry import BoundingBox
from semantic_digital_twin.world_description.world_entity import (
    Body,
    KinematicStructureEntity,
)

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


@dataclass
class _AllowObjectCollisions(AllowCollisionRule):
    """Suppresses collision avoidance for the object being grasped.

    Unlike AllowCollisionForBodies, this correctly repopulates allowed_collision_bodies
    on every world update so the exemption survives repeated update() calls.
    """

    _object_body: Body = field(kw_only=True)

    def _update(self, world) -> None:
        self.allowed_collision_bodies = {self._object_body}


class GraspSide(Enum):
    """Which face of the object to approach for grasping.

    CLOSEST selects the face most aligned with the current robot position (default).
    All other options force a specific face and bypass the gripper-width validity check.

    Axes are in the object's local frame:
      X_POS / X_NEG  – approach from the positive/negative X side
      Y_POS / Y_NEG  – approach from the positive/negative Y side
      TOP            – approach from above  (+Z)
      BOTTOM         – approach from below  (-Z)
    """

    CLOSEST = auto()
    X_POS = auto()
    X_NEG = auto()
    Y_POS = auto()
    Y_NEG = auto()
    TOP = auto()
    BOTTOM = auto()


HSR_GRIPPER_WIDTH = 0.15
AXIS_ALIGNMENT_THRESHOLD = 0.9


@dataclass(repr=False, eq=False)
class PickUp(Goal):
    grasp_magic: "GraspMagic" = field(kw_only=True)
    ft: bool = field(kw_only=True, default=False)
    simulated_execution: bool = field(default=True, kw_only=True)

    def expand(self, context: MotionStatechartContext) -> None:
        super().expand(context)
        robot = self.grasp_magic.manipulator._robot
        self.sequence = Sequence(
            [
                OpenHand(simulated_execution=self.simulated_execution),
                GraspingSequence(grasp_magic=self.grasp_magic),
                CloseHand(ft=self.ft, simulated_execution=self.simulated_execution),
            ]
        )
        self.add_node(self.sequence)
        arm_buffer = 0.01
        self.add_node(
            UpdateTemporaryCollisionRules(
                temporary_rules=[
                    *make_external_collision_rules(robot=robot, arm_buffer_zone=arm_buffer),
                    _AllowObjectCollisions(
                        _object_body=self.grasp_magic.object_geometry
                    ),
                ]
            )
        )
        # self.add_node(SelfCollisionAvoidance(robot=robot))
        self.add_node(ExternalCollisionAvoidance(robot=robot))

    def build(self, context: MotionStatechartContext) -> NodeArtifacts:
        artifacts = super().build(context)
        artifacts.observation = self.sequence.observation_variable
        return artifacts


@dataclass(repr=False, eq=False)
class GraspMagic(ABC):
    manipulator: ParallelGripper = field(kw_only=True)
    object_geometry: Body = field(kw_only=True)
    gripper_width: float = field(kw_only=True, default=HSR_GRIPPER_WIDTH)
    gripper_vertical: Optional[bool] = field(default=True, kw_only=True)
    preferred_side: GraspSide = field(default=GraspSide.CLOSEST, kw_only=True)
    pre_grasp_distance: float = field(default=0.15, kw_only=True)

    @abstractmethod
    def get_grasp_sequence(
            self, context: MotionStatechartContext
    ) -> Tuple[
        Tuple[CartesianPosition, "Parallel"], Tuple[CartesianPosition, "Parallel"]
    ]:
        """Returns ((pre_grasp_cart, pre_grasp_align), (grasp_cart, grasp_align))."""
        pass

    def _select_optimal_grasp_axis(
            self, context: MotionStatechartContext, obj_bbox: BoundingBox, obj_to_robot: Vector3
    ) -> Tuple[Vector3, Optional[float]]:
        """Returns (grasp_axis, forced_sign). forced_sign is ±1 for explicit sides, None for CLOSEST."""
        if self.preferred_side != GraspSide.CLOSEST:
            _side_map = {
                GraspSide.X_POS: ("x", +1.0),
                GraspSide.X_NEG: ("x", -1.0),
                GraspSide.Y_POS: ("y", +1.0),
                GraspSide.Y_NEG: ("y", -1.0),
                GraspSide.TOP: ("z", +1.0),
                GraspSide.BOTTOM: ("z", -1.0),
            }
            axis_name, forced_sign = _side_map[self.preferred_side]
            local_axis = {"x": Vector3.X, "y": Vector3.Y, "z": Vector3.Z}[axis_name](
                self.object_geometry
            )
            local_axis.reference_frame = self.object_geometry
            local_axis.scale(1)
            logger.debug(
                f"preferred_side={self.preferred_side.value}: axis={axis_name}, sign={forced_sign}"
            )
            return local_axis, forced_sign

        world_z = Vector3.Z(context.world.root)
        candidate_faces = [
            (Vector3.X(self.object_geometry), "x"),
            (Vector3.Y(self.object_geometry), "y"),
            (Vector3.Z(self.object_geometry), "z"),
        ]
        valid_faces = []
        for local_axis, axis_name in candidate_faces:
            axis_in_world = context.world.transform(
                spatial_object=local_axis, target_frame=context.world.root
            )
            axis_in_world.reference_frame = context.world.root

            if axis_name == "x":
                perp_dim1, perp_dim2 = obj_bbox.width, obj_bbox.height
                perp_axis1, perp_axis2 = Vector3.Y(self.object_geometry), Vector3.Z(
                    self.object_geometry
                )
            elif axis_name == "y":
                perp_dim1, perp_dim2 = obj_bbox.depth, obj_bbox.height
                perp_axis1, perp_axis2 = Vector3.X(self.object_geometry), Vector3.Z(
                    self.object_geometry
                )
            else:
                perp_dim1, perp_dim2 = obj_bbox.depth, obj_bbox.width
                perp_axis1, perp_axis2 = Vector3.X(self.object_geometry), Vector3.Y(
                    self.object_geometry
                )

            perp1_world = context.world.transform(
                spatial_object=perp_axis1, target_frame=context.world.root
            )
            perp2_world = context.world.transform(
                spatial_object=perp_axis2, target_frame=context.world.root
            )

            if abs(perp1_world.dot(world_z)) > abs(perp2_world.dot(world_z)):
                vertical_dim, horizontal_dim = perp_dim1, perp_dim2
            else:
                vertical_dim, horizontal_dim = perp_dim2, perp_dim1

            graspable_dim = horizontal_dim if self.gripper_vertical else vertical_dim
            if graspable_dim <= self.gripper_width:
                valid_faces.append((local_axis, axis_in_world, graspable_dim))

        if not valid_faces:
            raise Exception(
                f"No valid grasp face found. gripper_width={self.gripper_width}, "
                f"gripper_vertical={self.gripper_vertical}, "
                f"dims: w={obj_bbox.width:.3f} d={obj_bbox.depth:.3f} h={obj_bbox.height:.3f}"
            )

        grasp_axis = max(valid_faces, key=lambda x: abs(x[1].dot(obj_to_robot)))[0]
        grasp_axis.reference_frame = self.object_geometry
        grasp_axis.scale(1)
        return grasp_axis, None

    def _compute_grasp_geometry(self, context: MotionStatechartContext) -> Tuple[
        HomogeneousTransformationMatrix,
        Body,
        BoundingBox,
        Vector3,
        Vector3,
        Optional[float],
    ]:
        obj_pose = self.object_geometry.global_pose
        tool_frame = self.manipulator.tool_frame
        obj_bbox = self.object_geometry.collision.as_bounding_box_collection_in_frame(
            self.object_geometry
        ).bounding_box()
        obj_to_robot = (
                self.manipulator.tool_frame.global_pose.to_position()
                - obj_pose.to_position()
        )
        obj_to_robot.scale(1)
        grasp_axis, forced_sign = self._select_optimal_grasp_axis(
            context, obj_bbox, obj_to_robot
        )
        logger.debug(f"grasp_axis: {grasp_axis.to_np()}, forced_sign: {forced_sign}")
        return obj_pose, tool_frame, obj_bbox, obj_to_robot, grasp_axis, forced_sign

    def _compute_offset_along_axis(
            self, obj_bbox: BoundingBox, grasp_axis: Vector3, additional_offset: float = 0.0
    ) -> float:
        if abs(grasp_axis.x) > AXIS_ALIGNMENT_THRESHOLD:
            half_extent = obj_bbox.depth / 2.0
        elif abs(grasp_axis.y) > AXIS_ALIGNMENT_THRESHOLD:
            half_extent = obj_bbox.width / 2.0
        else:
            half_extent = obj_bbox.height / 2.0
        return half_extent + additional_offset

    def _compute_position_along_axis(
            self,
            context: MotionStatechartContext,
            obj_pose: HomogeneousTransformationMatrix,
            obj_bbox: BoundingBox,
            grasp_axis: Vector3,
            obj_to_robot: Vector3,
            additional_offset: float = 0.0,
            forced_sign: Optional[float] = None,
    ) -> Point3:
        grasp_axis_world = context.world.transform(
            spatial_object=grasp_axis, target_frame=context.world.root
        )
        grasp_axis_world.reference_frame = context.world.root
        if forced_sign is not None:
            approach_sign = forced_sign
        else:
            approach_sign = 1.0 if grasp_axis_world.dot(obj_to_robot) >= 0.0 else -1.0
        offset_distance = self._compute_offset_along_axis(
            obj_bbox, grasp_axis, additional_offset
        )
        position = obj_pose.to_position() + grasp_axis_world * (
                offset_distance * approach_sign
        )
        position.reference_frame = context.world.root
        return position

    def _get_orientation_nodes(
            self,
            context: MotionStatechartContext,
            tool_frame: KinematicStructureEntity,
            grasp_axis: Vector3,
            forced_sign: Optional[float],
            obj_to_robot: Vector3,
            obj_bbox: Optional[BoundingBox] = None,
            weight: float = DefaultWeights.WEIGHT_ABOVE_CA,
    ) -> List[MotionStatechartNode]:
        grasp_axis_world = context.world.transform(grasp_axis, context.world.root)
        grasp_axis_world.reference_frame = context.world.root
        approach_sign = (
            forced_sign
            if forced_sign is not None
            else (1.0 if grasp_axis_world.dot(obj_to_robot) >= 0.0 else -1.0)
        )

        # Tool Z points toward the object (opposite of the outward approach direction).
        z_tool = grasp_axis_world * (-approach_sign)
        z_tool.reference_frame = context.world.root

        world_z = Vector3.Z(context.world.root)
        if self.preferred_side in (GraspSide.TOP, GraspSide.BOTTOM):
            # Align X with the longer horizontal object axis so the opening (Y) faces
            # the shorter dimension and the object fits within the gripper width.
            obj_x = context.world.transform(
                Vector3.X(self.object_geometry), context.world.root
            )
            obj_y = context.world.transform(
                Vector3.Y(self.object_geometry), context.world.root
            )
            obj_x.reference_frame = context.world.root
            obj_y.reference_frame = context.world.root
            x_tool = (
                obj_x
                if (obj_bbox is None or obj_bbox.depth >= obj_bbox.width)
                else obj_y
            )
        elif self.gripper_vertical is False:
            # Y should align with world Z.  from_vectors computes Y = Z × X,
            # so X = world_Z × z_tool satisfies that.
            x_tool = world_z.cross(z_tool)
            x_tool.reference_frame = context.world.root
        else:
            # gripper_vertical=True or None: X aligned with world Z.
            x_tool = world_z

        goal_orientation = RotationMatrix.from_vectors(
            x=x_tool, z=z_tool, reference_frame=context.world.root
        )
        return [
            CartesianOrientation(
                root_link=context.world.root,
                tip_link=tool_frame,
                goal_orientation=goal_orientation,
                weight=weight,
                name="grasp_orientation",
            )
        ]


@dataclass(repr=False, eq=False)
class BoxGraspMagic(GraspMagic):
    def get_grasp_sequence(
            self, context: MotionStatechartContext
    ) -> Tuple[Tuple[CartesianPosition, Parallel], Tuple[CartesianPosition, Parallel]]:
        obj_pose, tool_frame, obj_bbox, obj_to_robot, grasp_axis, forced_sign = (
            self._compute_grasp_geometry(context)
        )

        pre_grasp_point = self._compute_position_along_axis(
            context,
            obj_pose,
            obj_bbox,
            grasp_axis,
            obj_to_robot,
            self.pre_grasp_distance,
            forced_sign=forced_sign,
        )
        logger.debug(f"pre_grasp_point: {pre_grasp_point.to_np()}")
        pre_cart = CartesianPosition(
            root_link=context.world.root,
            tip_link=tool_frame,
            goal_point=pre_grasp_point,
            name="pre_grasp_position",
        )
        pre_align = Parallel(
            self._get_orientation_nodes(
                context,
                tool_frame,
                grasp_axis,
                forced_sign,
                obj_to_robot,
                obj_bbox=obj_bbox,
                weight=DefaultWeights.WEIGHT_BELOW_CA,
            )
        )

        grasp_point = self._compute_position_along_axis(
            context,
            obj_pose,
            obj_bbox,
            grasp_axis,
            obj_to_robot,
            additional_offset=-0.05,
            forced_sign=forced_sign,
        )
        logger.debug(f"grasp_point: {grasp_point.to_np()}")
        grasp_cart = CartesianPosition(
            root_link=context.world.root,
            tip_link=tool_frame,
            goal_point=grasp_point,
            name="grasp_position",
        )
        grasp_align = Parallel(
            self._get_orientation_nodes(
                context,
                tool_frame,
                grasp_axis,
                forced_sign,
                obj_to_robot,
                obj_bbox=obj_bbox,
            )
        )

        return (pre_cart, pre_align), (grasp_cart, grasp_align)


@dataclass(repr=False, eq=False)
class CylinderGraspMagic(GraspMagic):
    def get_grasp_sequence(
            self, context: MotionStatechartContext
    ) -> Tuple[Tuple[CartesianPosition, Parallel], Tuple[CartesianPosition, Parallel]]:
        obj_pose = self.object_geometry.global_pose
        tool_frame = self.manipulator.tool_frame

        cylinder_shape = next(
            s for s in self.object_geometry.collision.shapes if isinstance(s, Cylinder)
        )
        radius = cylinder_shape.width / 2

        # Cylinder axis in world frame
        cyl_axis_world = context.world.transform(
            Vector3.Z(self.object_geometry), context.world.root
        )
        cyl_axis_world.reference_frame = context.world.root

        # Project obj_to_robot onto the plane perpendicular to the cylinder axis
        obj_to_robot = (
                self.manipulator.tool_frame.global_pose.to_position()
                - obj_pose.to_position()
        )
        obj_to_robot.scale(1)
        radial_dir = obj_to_robot - cyl_axis_world * obj_to_robot.dot(cyl_axis_world)
        radial_dir.scale(1)
        radial_dir.reference_frame = context.world.root

        cyl_center = obj_pose.to_position()

        pre_grasp_point = cyl_center + radial_dir * (radius + self.pre_grasp_distance)
        pre_grasp_point.reference_frame = context.world.root
        logger.debug(f"cylinder pre_grasp_point: {pre_grasp_point.to_np()}")
        pre_cart = CartesianPosition(
            root_link=context.world.root,
            tip_link=tool_frame,
            goal_point=pre_grasp_point,
            name="pre_grasp_position",
        )
        pre_align = Parallel(
            self._get_orientation_nodes(
                context,
                tool_frame,
                radial_dir,
                1.0,
                obj_to_robot,
                weight=DefaultWeights.WEIGHT_BELOW_CA,
            )
        )

        grasp_point = cyl_center + radial_dir * (radius - 0.05)
        grasp_point.reference_frame = context.world.root
        logger.debug(f"cylinder grasp_point: {grasp_point.to_np()}")
        grasp_cart = CartesianPosition(
            root_link=context.world.root,
            tip_link=tool_frame,
            goal_point=grasp_point,
            name="grasp_position",
        )
        grasp_align = Parallel(
            self._get_orientation_nodes(
                context,
                tool_frame,
                radial_dir,
                1.0,
                obj_to_robot,
            )
        )

        return (pre_cart, pre_align), (grasp_cart, grasp_align)


@dataclass(repr=False, eq=False)
class _GraspPhase(Goal):
    _cart: CartesianPosition = field(kw_only=True)
    _align: Parallel = field(kw_only=True)
    _wait_for_align: bool = field(kw_only=True, default=True)

    def expand(self, context: MotionStatechartContext) -> None:
        self.add_node(self._cart)
        self.add_node(self._align)
        self.add_node(ExternalCollisionAvoidance())

    def build(self, context: MotionStatechartContext) -> NodeArtifacts:
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
    grasp_magic: GraspMagic = field(kw_only=True)

    def expand(self, context: MotionStatechartContext) -> None:
        robot = self.grasp_magic.manipulator._robot
        (pre_cart, pre_align), (grasp_cart, grasp_align) = (
            self.grasp_magic.get_grasp_sequence(context)
        )
        self._seq = Sequence(
            [
                _GraspPhase(_cart=pre_cart, _align=pre_align, _wait_for_align=True),
                _GraspPhase(
                    _cart=grasp_cart, _align=grasp_align, _wait_for_align=False
                ),
            ]
        )
        self.add_node(self._seq)

    def build(self, context: MotionStatechartContext) -> NodeArtifacts:
        artifacts = super().build(context)
        artifacts.observation = self._seq.observation_variable
        return artifacts


@dataclass(repr=False, eq=False)
class PullUp(Goal):
    manipulator: ParallelGripper = field(kw_only=True)
    object_geometry: Body = field(kw_only=True)
    ft: bool = field(kw_only=True, default=False)

    def expand(self, context: MotionStatechartContext) -> None:
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
            tip_link=self.object_geometry,
            goal_orientation=self.object_geometry.global_pose.to_rotation_matrix(),
        )
        self.add_node(self._cart_position)
        self.add_node(self._keep_orientation)

    def build(self, context: MotionStatechartContext) -> NodeArtifacts:
        artifacts = super().build(context)
        if self.ft:
            artifacts.observation = trinary_logic_and(
                self._ft.observation_variable, self._cart_position.observation_variable
            )
        else:
            artifacts.observation = self._cart_position.observation_variable
        return artifacts
