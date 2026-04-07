from __future__ import annotations

import logging

import rclpy

import pycram_suturo_demos.helper_methods_and_useful_classes.nlp_human_robot_interaction as hri
from time import sleep

from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms
from pycram_suturo_demos.pycram_basic_hsr_demos.gripper_open_close_demo import (
    GripperActionClient,
)
from pycram.language import SequentialPlan
from pycram.motion_executor import ExecutionEnvironment, simulated_robot, real_robot
from pycram.robot_plans import (
    BaseMotion,
    MoveTCPMotion,
    HandoverActionDescription,
    ParkArmsActionDescription,
)
from semantic_digital_twin.datastructures.joint_state import JointState
from semantic_digital_twin.world import World
from dataclasses import dataclass, field


from pycram_suturo_demos.helper_methods_and_useful_classes.A_robot_setup import (
    robot_setup,
)


def initialization(simulation: bool = True):
    logger = logging.getLogger(__name__)

    result = robot_setup(simulation=simulation)
    rclpy_node, world, robot_view, context = (
        result.node,
        result.world,
        result.robot_view,
        result.context,
    )
    return rclpy_node, world, robot_view, context


def countdown(n, node: hri.TalkingNode):
    while n > 0:
        node.pub(str(n))
        sleep(2)
        n -= 1


def take_object_from_human():
    talk = hri.TalkingNode()
    gripper_open = 0.8
    gripper_close = -0.8

    grippy = GripperActionClient()
    grippy.send_goal(effort=gripper_open)

    talk.pub("Please put the object into my gripper. Closing gripper in ")
    sleep(5)
    countdown(5, talk)
    sleep(1)
    grippy.send_goal(effort=gripper_close)
    sleep(3)


def give_object_to_human():
    talk = hri.TalkingNode()
    gripper_open = 0.8

    grippy = GripperActionClient()

    talk.pub("Please take the object from my gripper. Opening gripper in ")
    sleep(5)
    countdown(5, talk)
    sleep(1)
    grippy.send_goal(effort=gripper_open)
    sleep(3)


# human has object and robot takes object from human
def handover_human_robot():
    with_giskard = True
    rclpy.init()

    if with_giskard:
        SIMULATED = False
        robot_type: ExecutionEnvironment = simulated_robot if SIMULATED else real_robot

        rclpy_node, world, robot_view, context = initialization(simulation=SIMULATED)

        with robot_type:

            SequentialPlan(
                context,
                HandoverActionDescription(world=world),
            ).perform()

            take_object_from_human()
            # give_object_to_human()

            print("Parking arms")
            SequentialPlan(
                context,
                ParkArmsActionDescription(Arms.LEFT),
            ).perform()
            print("Done")
    else:
        take_object_from_human()
        # give_object_to_human()
    sleep(5)


# robot has object and human takes object from robot
def handover_robot_human():
    with_giskard = True
    rclpy.init()

    if with_giskard:
        SIMULATED = False
        robot_type: ExecutionEnvironment = simulated_robot if SIMULATED else real_robot

        rclpy_node, world, robot_view, context = initialization(simulation=SIMULATED)

        with robot_type:

            SequentialPlan(
                context,
                HandoverActionDescription(world=world),
            ).perform()

            give_object_to_human()

            print("Parking arms")
            SequentialPlan(
                context,
                ParkArmsActionDescription(Arms.LEFT),
            ).perform()
            print("Done")
    else:
        give_object_to_human()
    sleep(5)


def handover_robot_human_no_init(*, simulated: bool = False, context: Context = None):
    from pycram.datastructures.dataclasses import Context

    SIMULATED = simulated
    robot_type: ExecutionEnvironment = simulated_robot if SIMULATED else real_robot

    with robot_type:
        SequentialPlan(
            context,
            HandoverActionDescription(world=context.world),
        ).perform()

        give_object_to_human()

        print("Parking arms")
        SequentialPlan(
            context,
            ParkArmsActionDescription(Arms.LEFT),
        ).perform()
        print("Done")
    sleep(5)


# human has object and robot takes object from human
def handover_human_robot_no_init(*, simulated: bool = False, context: Context = None):
    with_giskard = True
    rclpy.init()
    world = context.world

    SIMULATED = simulated
    robot_type: ExecutionEnvironment = simulated_robot if SIMULATED else real_robot

    with robot_type:

        SequentialPlan(
            context,
            HandoverActionDescription(world=world),
        ).perform()

        take_object_from_human()

        print("Parking arms")
        SequentialPlan(
            context,
            ParkArmsActionDescription(Arms.LEFT),
        ).perform()
        print("Done")

    sleep(5)


def main():
    handover_human_robot()


if __name__ == "__main__":
    main()
