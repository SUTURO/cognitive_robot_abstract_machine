from dataclasses import dataclass, field
from enum import Enum

from giskardpy.motion_statechart.context import BuildContext
from giskardpy.motion_statechart.graph_node import Goal, NodeArtifacts
from giskardpy.motion_statechart.ros2_nodes.gripper_command import GripperCommandTask
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList
from semantic_digital_twin.datastructures.joint_state import JointState


class HSRGripper(Enum):
    open_gripper = 1.23
    close_gripper = 0


@dataclass(repr=False, eq=False)
class CloseHand(Goal):
    ft: bool = field(kw_only=True, default=False)
    simulated_execution: bool = field(kw_only=True, default=True)

    def expand(self, context: BuildContext) -> None:
        if self.simulated_execution:
            self.close_gripper = JointPositionList(
                goal_state=JointState.from_str_dict(
                    {"hand_motor_joint": HSRGripper.close_gripper.value}, context.world
                )
            )
        else:
            self.close_gripper = GripperCommandTask(
                action_topic="/gripper_controller/grasp",
                effort=-0.8,
            )
        self.add_node(self.close_gripper)

    def build(self, context: BuildContext) -> NodeArtifacts:
        artifacts = super().build(context)
        artifacts.observation = self.close_gripper.observation_variable
        return artifacts


@dataclass(repr=False, eq=False)
class OpenHand(Goal):
    simulated_execution: bool = field(kw_only=True, default=True)

    def expand(self, context: BuildContext) -> None:
        if self.simulated_execution:
            self.open_gripper = JointPositionList(
                goal_state=JointState.from_str_dict(
                    {"hand_motor_joint": HSRGripper.open_gripper.value}, context.world
                )
            )
        else:
            self.open_gripper = GripperCommandTask(
                action_topic="/gripper_controller/grasp", effort=0.8
            )
        self.add_node(self.open_gripper)

    def build(self, context: BuildContext) -> NodeArtifacts:
        artifacts = super().build(context)
        artifacts.observation = self.open_gripper.observation_variable
        return artifacts
