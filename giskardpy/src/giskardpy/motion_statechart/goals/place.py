from dataclasses import dataclass, field
from typing import Union
from giskardpy.data_types.exceptions import ForceTorqueSaysNoException
from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.data_types import DefaultWeights
from giskardpy.motion_statechart.goals.collision_avoidance import (
    ExternalCollisionAvoidance,
    ExternalCollisionDistanceMonitor,
    SelfCollisionAvoidance,
    UpdateTemporaryCollisionRules,
    make_external_collision_rules,
)
from giskardpy.motion_statechart.goals.pick_up import CloseHand, OpenHand, _AllowObjectCollisions
from giskardpy.motion_statechart.goals.templates import Sequence
from giskardpy.motion_statechart.graph_node import Goal, NodeArtifacts, CancelMotion
from giskardpy.motion_statechart.monitors.monitors import LocalMinimumReached
from giskardpy.motion_statechart.ros2_nodes.force_torque_monitor import (
    ForceImpactMonitor,
)
from giskardpy.motion_statechart.tasks.align_planes import AlignPlanes
from giskardpy.motion_statechart.tasks.cartesian_tasks import (
    CartesianPosition,
    CartesianPose,
)
from krrood.symbolic_math.symbolic_math import trinary_logic_or, trinary_logic_and, trinary_logic_not
from semantic_digital_twin.robots.abstract_robot import AbstractRobot, Manipulator
from semantic_digital_twin.spatial_types import (
    Vector3,
    Point3,
    HomogeneousTransformationMatrix,
)
from semantic_digital_twin.world_description.world_entity import (
    Body,
    KinematicStructureEntity,
)


class PlacementNotReachableException(Exception):
    pass


@dataclass(repr=False, eq=False)
class Place(Goal):
    """
    Assumes the object is already attached to the tool frame and the gripper is closed.
    goal: HomogeneousTransformationMatrix → full pose control; Point3 → position-only with z-up alignment.
    """

    manipulator: Manipulator = field(kw_only=True)
    object_geometry: Body = field(kw_only=True)
    goal: Union[HomogeneousTransformationMatrix, Point3] = field(kw_only=True)
    ft: bool = field(kw_only=True, default=False)
    simulated: bool = field(default=True, kw_only=True)
    pre_place_distance: float = field(default=0.15, kw_only=True)
    _motion_sequence: Sequence = field(init=False)

    def expand(self, context: MotionStatechartContext) -> None:
        super().expand(context)
        robot = self.manipulator._robot
        # Note: Retracting separate from placing
        self._motion_sequence = Sequence([
            ApproachPlacement(
                manipulator=self.manipulator,
                object_geometry=self.object_geometry,
                goal=self.goal,
                ft=self.ft,
                pre_place_distance=self.pre_place_distance,
            ),
            OpenHand(simulated_execution=self.simulated),
        ])
        self.add_node(self._motion_sequence)
        arm_buffer = 0.025 # min(0.05, max(self.object_geometry.collision.scale.z / 2 - 0.01, 0.01))
        self.add_node(
            UpdateTemporaryCollisionRules(
                temporary_rules=[
                    *make_external_collision_rules(robot=robot, arm_buffer_zone=arm_buffer),
                    _AllowObjectCollisions(_object_body=self.object_geometry),
                ]
            )
        )
        # self.add_node(SelfCollisionAvoidance(robot=robot))
        self.add_node(ExternalCollisionAvoidance(robot=robot))

    def build(self, context: MotionStatechartContext) -> NodeArtifacts:
        artifacts = super().build(context)
        artifacts.observation = self._motion_sequence.observation_variable
        return artifacts


@dataclass(repr=False, eq=False)
class ApproachPlacement(Goal):
    # Assumes object is attached to tool frame
    manipulator: Manipulator = field(kw_only=True)
    object_geometry: Body = field(kw_only=True)
    goal: Union[HomogeneousTransformationMatrix, Point3] = field(kw_only=True)
    ft: bool = field(kw_only=True, default=False)
    pre_place_distance: float = field(default=0.1, kw_only=True)

    def expand(self, context: MotionStatechartContext) -> None:
        super().expand(context)

        if self.ft:
            self.ft_monitor = ForceImpactMonitor(threshold=5, topic_name="ft_irgendwas")
            self.add_node(self.ft_monitor)

        if isinstance(self.goal, HomogeneousTransformationMatrix):
            goal_pose = self.goal
            if goal_pose.reference_frame != context.world.root:
                goal_pose = context.world.transform(
                    spatial_object=goal_pose, target_frame=context.world.root
                )

            pre_tool_pose = HomogeneousTransformationMatrix.from_point_rotation_matrix(
                point=goal_pose.to_position()
                + Vector3(0, 0, self.pre_place_distance, reference_frame=context.world.root),
                rotation_matrix=self.object_geometry.global_pose.to_rotation_matrix(),  # goal_pose.to_rotation_matrix(),
                reference_frame=context.world.root,
            )
            pre_pose_goal = CartesianPose(
                root_link=context.world.root,
                tip_link=self.object_geometry,
                goal_pose=pre_tool_pose,
            )
            self.object_goal = CartesianPose(
                root_link=context.world.root,
                tip_link=self.object_geometry,
                goal_pose=goal_pose,
                reference_linear_velocity=0.1,
            )
            self.add_node(Sequence([pre_pose_goal, self.object_goal]))
        elif isinstance(self.goal, Point3):
            goal_point = context.world.transform(
                spatial_object=self.goal, target_frame=context.world.root
            )
            pre_point = goal_point + Vector3(
                0, 0, self.pre_place_distance, reference_frame=context.world.root
            )
            pre_point.reference_frame = context.world.root

            # Z-up throughout: align object Z with world Z, yaw remains free
            z_up = AlignPlanes(
                root_link=context.world.root,
                tip_link=self.object_geometry,
                tip_normal=Vector3.Z(self.object_geometry),
                goal_normal=Vector3.Z(context.world.root),
            )
            pre_pos = CartesianPosition(
                root_link=context.world.root,
                tip_link=self.object_geometry,
                goal_point=pre_point,
            )
            self.object_goal = CartesianPosition(
                root_link=context.world.root,
                tip_link=self.object_geometry,
                goal_point=goal_point,
                reference_velocity=0.1,
            )
            self.add_node(z_up)
            self.add_node(Sequence([pre_pos, self.object_goal]))
        else:
            raise TypeError(
                f"goal must be HomogeneousTransformationMatrix or Point3, got {type(self.goal)}"
            )

        # stuck = LocalMinimumReached()
        # self.add_node(stuck)
        # cancel = CancelMotion(exception=PlacementNotReachableException("Placement position is not reachable"))
        # cancel.start_condition = trinary_logic_and(
        #     stuck.observation_variable,
        #     trinary_logic_not(self.object_goal.observation_variable),
        # )
        # self.add_node(cancel)

    def build(self, context: MotionStatechartContext) -> NodeArtifacts:
        artifacts = super().build(context)
        if self.ft:
            artifacts.observation = trinary_logic_or(
                self.ft_monitor.observation_variable,
                self.object_goal.observation_variable,
            )
        else:
            artifacts.observation = self.object_goal.observation_variable
        return artifacts


@dataclass(repr=False, eq=False)
class Retracting(Goal):
    """
    Retracts the tool frame of a manipulator by a certain distance.
    """

    manipulator: Manipulator = field(kw_only=True)
    distance: float = field(default=0.15, kw_only=True)
    velocity: float = field(default=0.1, kw_only=True)
    weight: float = field(default=DefaultWeights.WEIGHT_ABOVE_CA, kw_only=True)
    ft: bool = field(default=False, kw_only=True)

    def expand(self, context: MotionStatechartContext) -> None:
        tip_link = self.manipulator.tool_frame
        root_link = context.world.root

        if self.ft:
            # When retracting you shouldnt bump into anything
            self._ft = ForceImpactMonitor(threshold=5, topic_name="ft_irgendwas")
            self._cm = CancelMotion.when_true(self._ft)
            self.add_node(self._ft)
            self.add_node(self._cm)

        goal_point = Point3(0, 0, -self.distance, reference_frame=tip_link)

        self.cart_pos = CartesianPosition(
            root_link=root_link,
            tip_link=tip_link,
            goal_point=goal_point,
            weight=self.weight,
        )

        self.add_node(self.cart_pos)

    def build(self, context: MotionStatechartContext) -> NodeArtifacts:
        artifacts = super().build(context)
        artifacts.observation = self.cart_pos.observation_variable
        return artifacts
