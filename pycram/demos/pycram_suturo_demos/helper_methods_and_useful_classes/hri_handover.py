from __future__ import annotations

import logging

import rclpy

import nlp_human_robot_interaction as hri
from time import sleep
from pycram_suturo_demos.helper_methods_and_useful_classes.robot_setup import (
    robot_setup,
)
from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot, simulated_robot
from pycram.robot_plans import (
    ParkArmsActionDescription,
    MoveGripperMotion,
    NavigateAction,
    NavigateActionDescription,
)
from pycram.datastructures.enums import Arms
from semantic_digital_twin.datastructures.definitions import GripperState


########################################################################################################################

from dataclasses import dataclass, field
from datetime import timedelta

from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.world_description.connections import Connection6DoF
from semantic_digital_twin.world_description.world_entity import Body
from typing_extensions import Union, Optional, Type, Any, Iterable

from pycram.robot_plans.actions.core.pick_up import ReachActionDescription, PickUpAction
from pycram.config.action_conf import ActionConfig
from pycram.robot_plans.motions.gripper import (
    MoveTCPMotion,
    MoveGripperMotion,
    ReachMotion,
)
from pycram.datastructures.enums import (
    Arms,
    ApproachDirection,
    VerticalAlignment,
)
from pycram.datastructures.grasp import GraspDescription
from pycram.datastructures.partial_designator import PartialDesignator
from pycram.datastructures.pose import PoseStamped
from pycram.failures import ObjectNotPlacedAtTargetLocation, ObjectStillInContact
from pycram.language import SequentialPlan
from pycram.view_manager import ViewManager
from pycram.robot_plans.actions.base import ActionDescription
from pycram.utils import translate_pose_along_local_axis
from pycram.validation.error_checkers import PoseErrorChecker
from pycram.visualization import plot_rustworkx_interactive


@dataclass
class GripperAction(ActionDescription):
    """
    Places an Object at a position using an arm.
    """

    """
    Pose in the world at which the object should be placed
    """
    arm: Arms
    """
    Arm that is currently holding the object
    """
    _pre_perform_callbacks = []
    """
    List to save the callbacks which should be called before performing the action.
    """
    gripper_state: GripperState

    def __post_init__(self):
        super().__post_init__()

    def execute(self) -> None:

        arm = ViewManager.get_arm_view(self.arm, self.robot_view)
        manipulator = arm.manipulator

        previous_pick = self.plan.get_previous_node_by_designator_type(
            self.plan_node, PickUpAction
        )
        previous_grasp = (
            previous_pick.designator_ref.grasp_description
            if previous_pick
            else GraspDescription(
                ApproachDirection.FRONT, VerticalAlignment.NoAlignment, manipulator
            )
        )

        SequentialPlan(
            self.context,
            MoveGripperMotion(self.gripper_state, self.arm),
        ).perform()

    @classmethod
    def description(
        cls,
        arm: Union[Iterable[Arms], Arms],
        gripper_state: GripperState,
    ) -> PartialDesignator[GripperAction]:
        return PartialDesignator[GripperAction](
            GripperAction, arm=arm, gripper_state=gripper_state
        )


GripperActionDescription = GripperAction.description
########################################################################################################################

logger = logging.getLogger(__name__)

rclpy.init()
result = robot_setup(True)
world, robot_view, context = (result.world, result.robot_view, result.context)

simulated = False

world_root = getattr(world, "root")

plan_base = SequentialPlan(context, ParkArmsActionDescription(Arms.LEFT))

if simulated:
    robot = simulated_robot
else:
    robot = real_robot


def countdown(n, node: hri.TalkingNode):
    while n > 0:
        node.pub(str(n))
        sleep(1.5)
        n -= 1


def take_object_from_human(goal_pose: PoseStamped):
    talk = hri.TalkingNode()

    # mplan = SequentialPlan(context, NavigateActionDescription(goal_pose, keep_joint_states=True))
    # mmplan = SequentialPlan(context, ParkArmsActionDescription(Arms.LEFT),
    # GripperActionDescription(arm=Arms.LEFT, gripper_state=GripperState.OPEN))

    plan = SequentialPlan(
        context,
        ParkArmsActionDescription(Arms.LEFT),
        MoveGripperMotion(motion=GripperState.OPEN, gripper=Arms.LEFT),
    )
    with robot:
        plan.perform()
        # mmplan.perform()

    talk.pub("Please put the object into my gripper. Closing gripper in ")
    countdown(5, talk)

    plan2 = SequentialPlan(
        context,
        MoveGripperMotion(motion=GripperState.CLOSE, gripper=Arms.LEFT),
        ParkArmsActionDescription(Arms.LEFT),
    )

    with robot:
        plan2.perform()

    logger.info("Finished taking object successfully.")


def give_object_to_human():
    print("give object to human")


def demo(step: int):
    talk = hri.TalkingNode()
    talk.pub("I am talking")


def main():
    goal_pose = PoseStamped.from_list(
        position=[1, 1, 1], orientation=[0, 0, 1, 1], frame=world_root
    )
    take_object_from_human(goal_pose)


if __name__ == "__main__":
    main()
