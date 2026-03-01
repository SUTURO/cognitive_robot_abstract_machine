from __future__ import annotations

import logging
import rclpy

import semantic_digital_twin
from demos.pycram_suturo_demos.pycram_basic_hsr_demos.move_demo import move_demo
from demos.pycram_suturo_demos.pycram_basic_hsr_demos.pickup_demo_marc import (
    pickup_demo,
)
from pycram.datastructures.pose import PoseStamped

"""
was brauche ich:
[ ] - Home position
[ ] - Navigate to table
[ ] - Retrieve item from world
[ ] - Handle if item not existing
[ ] - PickUp
[ ] - Attach to gripper
[ ] - Drive back

"""

from dataclasses import field

from demos.pycram_suturo_demos.helper_methods_and_useful_classes.robot_setup import (
    robot_setup,
)


def initialization(
    simulation: bool = True, with_perception: bool = False, object_name: str = ""
):
    starting_position: PoseStamped = PoseStamped.from_list()
    logging.getLogger(semantic_digital_twin.world.__name__).setLevel(logging.WARN)

    logger = logging.getLogger(__name__)
    result = robot_setup(simulation=simulation, with_perception=with_perception)
    rclpy_node, world, robot_view, context = (
        result.node,
        result.world,
        result.robot_view,
        result.context,
    )
    return rclpy_node, world, robot_view, context


def main():
    rclpy.init()
    SIMULATED = True
    object_name = "milk.stl"
    with_perception = False

    rclpy_node, world, robot_view, context = initialization(
        simulation=SIMULATED, with_perception=with_perception
    )
    move_demo(world=world, context=context, target_pose="POPCORN_TABLE")
    pickup_demo(
        simulation=SIMULATED,
        hsrb_world=world,
        context=context,
        object_name=object_name,
        with_perception=with_perception,
    )
    move_demo(world=world, context=context, target_pose="ROBOT_START_POSE")

    pass


if __name__ == "__main__":
    main()
