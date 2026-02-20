from dataclasses import dataclass, field
from typing import Optional
from typing_extensions import Type

from giskardpy.motion_statechart.context import BuildContext
from giskardpy.motion_statechart.data_types import DefaultWeights
from giskardpy.motion_statechart.goals.pick_up import CloseHand
from giskardpy.motion_statechart.goals.templates import Sequence
from giskardpy.motion_statechart.graph_node import Goal, NodeArtifacts
from giskardpy.motion_statechart.ros2_nodes.gripper_command import GripperCommandTask
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
    cartesian_pose: Pose = field(kw_only=True)
    ft: bool = field(kw_only=True, default=False)

    def __post_init__(self):
        super().__post_init__()
        # Note: REtracting seperate from placing
        approach = ApproachParkingSpace()
        # close_gipper = CloseHand()
        close_gripper = GripperCommandTask(
            action_topic="/gripper_controller/grasp",
            effort=0.8,
        )
        retracting = Retracting()

        self.nodes.append(approach)
        self.nodes.append(close_gripper)
        self.nodes.append(retracting)


class ApproachParkingSpace(Goal):
    manipulator: Manipulator = field(kw_only=True)
    object_geometry: Body = field(kw_only=True)
    ft: bool = field(kw_only=True, default=False)

    def expand(self, context: BuildContext) -> None:
        super().expand(context)

    def build(self, context: BuildContext) -> NodeArtifacts:
        return super().build(context)


@dataclass(repr=False, eq=False)
class Retracting(Goal):
    """
    Retracts the tool frame of a manipulator by a certain distance.
    """

    # TODO mach besser den scheiß mit tool frame z achse in rückwärts und so
    manipulator: Manipulator = field(kw_only=True)
    distance: float = field(default=0.15, kw_only=True)
    velocity: float = field(default=0.1, kw_only=True)
    weight: float = field(default=DefaultWeights.WEIGHT_ABOVE_CA, kw_only=True)

    def expand(self, context: BuildContext) -> None:
        tip_link = self.manipulator.tool_frame
        root_link = context.world.root

        goal_point = Point3(0, 0, 0, reference_frame=tip_link)
        self.reference_frame.global_pose.to_position()
        goal_point.x += direction.x
        goal_point.y += direction.y
        goal_point.z += direction.z

        world_goal_point = context.world.transform(
            target_frame=root_link, spatial_object=goal_point
        )

        self.cart_pos = CartesianPosition(
            root_link=root_link,
            tip_link=tip_link,
            goal_point=goal_point,
            weight=self.weight,
        )

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
        self.add_node(self.cart_pos)

    def build(self, context: BuildContext) -> NodeArtifacts:
        artifacts = super().build(context)
        artifacts.observation = self.parallel.observation_variable
        artifacts.observation = self.cart_pos.observation_variable
        return artifacts
