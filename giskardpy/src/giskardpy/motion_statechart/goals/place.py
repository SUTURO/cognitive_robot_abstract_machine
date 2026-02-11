from dataclasses import dataclass, field
from giskardpy.data_types.exceptions import ForceTorqueSaysNoException
from giskardpy.motion_statechart.context import BuildContext
from giskardpy.motion_statechart.data_types import DefaultWeights
from giskardpy.motion_statechart.goals.pick_up import CloseHand
from giskardpy.motion_statechart.goals.templates import Sequence, Parallel
from giskardpy.motion_statechart.graph_node import Goal, NodeArtifacts, CancelMotion
from giskardpy.motion_statechart.ros2_nodes.force_torque_monitor import ForceImpactMonitor
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPosition, CartesianPose
from krrood.symbolic_math.symbolic_math import trinary_logic_or
from semantic_digital_twin.robots.abstract_robot import Manipulator
from semantic_digital_twin.spatial_types import (
    Vector3,
    Point3,
    HomogeneousTransformationMatrix,
)
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.world_entity import (
    Body,
    KinematicStructureEntity,
)


@dataclass(repr=False, eq=False)
class Place(Sequence):
    """
    Assumes the object is already attached to the tool frame and the gripper is closed
    """

    manipulator: Manipulator = field(kw_only=True)
    object_geometry: Body = field(kw_only=True)
    ft: bool = field(kw_only=True, default=False)
    goal_pose: HomogeneousTransformationMatrix = field(kw_only=True)
    simulated: bool = field(default=True, kw_only=True)

    def __post_init__(self):
        super().__post_init__()
        # Note: Retracting seperate from placing
        approach = ApproachPlacement(manipulator=self.manipulator, object_geometry=self.object_geometry,
                                     goal_pose=self.goal_pose, ft=self.ft)
        close_gripper = CloseHand(ft=self.ft, simulated_execution=self.simulated)
        # retracting = Retracting(manipulator=self.manipulator)

        self.nodes.append(approach)
        self.nodes.append(close_gripper)
        # self.nodes.append(retracting)


@dataclass(repr=False, eq=False)
class ApproachPlacement(Goal):
    # Assumes object is attached to tool frame
    manipulator: Manipulator = field(kw_only=True)
    object_geometry: Body = field(kw_only=True)
    goal_pose: HomogeneousTransformationMatrix = field(kw_only=True)
    ft: bool = field(kw_only=True, default=False)

    def expand(self, context: BuildContext) -> None:
        super().expand(context)

        if self.goal_pose.reference_frame != context.world.root:
            self.goal_pose = context.world.transform(spatial_object=self.goal_pose, target_frame=context.world.root)


        # tool_to_object = context.world.transform(spatial_object=self.object_geometry.global_pose,
        #                                          target_frame=self.manipulator.tool_frame)
        #
        # tool_goal_pose = self.goal_pose.dot(tool_to_object.inverse())

        pre_tool_pose = HomogeneousTransformationMatrix.from_point_rotation_matrix(
            point=self.goal_pose.to_position() + Vector3(0, 0, 0.2, reference_frame=context.world.root),
            rotation_matrix=self.goal_pose.to_rotation_matrix(),
            reference_frame=context.world.root
        )
        pre_pose_goal = CartesianPose(root_link=context.world.root, tip_link=self.manipulator.tool_frame,
                                      goal_pose=pre_tool_pose)

        self.object_goal = CartesianPose(root_link=context.world.root, tip_link=self.object_geometry,
                                         goal_pose=self.goal_pose,)
        if self.ft:
            # Detect when the object hits the surface
            self.ft_monitor = ForceImpactMonitor(threshold=5, topic_name="ft_irgendwas")
            self.add_node(self.ft_monitor)

        self.add_node(Sequence([pre_pose_goal, self.object_goal]))

    def build(self, context: BuildContext) -> NodeArtifacts:
        artifacts = super().build(context)
        if self.ft:
            artifacts.observation = trinary_logic_or(self.ft_monitor.observation_variable,
                                                     self.object_goal.observation_variable)
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

    def expand(self, context: BuildContext) -> None:
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

    def build(self, context: BuildContext) -> NodeArtifacts:
        artifacts = super().build(context)
        artifacts.observation = self.cart_pos.observation_variable
        return artifacts
