from __future__ import annotations

import logging
import rclpy

from pycram.datastructures.enums import Arms
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot
from pycram.robot_plans import ParkArmsActionDescription, MoveTorsoActionDescription
from pycram_suturo_demos.helper_methods_and_useful_classes.pickup_helper_methods import (
    attach_object_to_hsrb,
)
from pycram_suturo_demos.pycram_basic_hsr_demos.move_demo import move_demo
from pycram_suturo_demos.pycram_basic_hsr_demos.pickup_demo import (
    pickup_demo,
)
from pycram_suturo_demos.pycram_basic_hsr_demos.place_demo import place_demo
from pycram.datastructures.pose import PoseStamped
from semantic_digital_twin.datastructures.definitions import TorsoState

"""
was brauche ich:
[X] - Home position
[X] - Navigate to table
[X] - Retrieve item from world
[ ] - Handle if item not existing
[X] - PickUp
[X] - Attach to gripper
[X] - Drive back

"""

from dataclasses import field

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


def try_perceive_and_spawn(world):
    try:
        from pycram_suturo_demos.helper_methods_and_useful_classes.object_creation import (
            perceive_and_spawn_all_objects,
        )

        perceived_objects = perceive_and_spawn_all_objects(world=world)
    except ImportError:
        print("Could not import robokudo")
        perceived_objects = {}
    return perceived_objects


def main():
    rclpy.init()
    SIMULATED = False

    rclpy_node, world, robot_view, context = initialization(simulation=SIMULATED)
    object_name = "muesli_vitalis_box_nutmix2"

    # objects = {1: "milk.stl", 2: "breakfast_cereal.stl"}

    plan = SequentialPlan(
        context,
        # MoveTorsoActionDescription(TorsoState.LOW),
        ParkArmsActionDescription(Arms.BOTH),
    )

    with real_robot:
        plan.perform()
    # # move_demo(
    # #     simulated=SIMULATED,
    # #     world=world,
    # #     context=context,
    # #     target_pose="POPCORN_TABLE",
    # # )
    # perceived_objects = try_perceive_and_spawn(world)
    # object_to_pickup = world.get_body_by_name(object_name)
    #
    # object_to_pickup_point = world.get_body_by_name(
    #     object_name
    # ).global_pose.to_position()
    # # print(object_to_pickup_pose)
    # object_height = object_to_pickup.collision.scale.z / 2
    # # print(object_to_pickup)
    # pickup_demo(
    #     simulation=SIMULATED,
    #     context=context,
    #     object_to_pickup=object_to_pickup,
    # )
    # move_demo(
    #     simulated=SIMULATED, world=world, context=context, target_pose="POPCORN_TABLE"
    # )
    # # attach_object_to_hsrb(world=world, object_designator=object_to_pickup)
    # # change the coords accordingly
    # place_pose = PoseStamped.from_list(
    #     [1.4, 6.2, 0.52 + object_height], [0, 0, 0, 1], frame=world.root
    # )
    # place_demo(
    #     simulation=SIMULATED,
    #     place_pose=place_pose,
    #     hsrb_world=world,
    #     context=context,
    #     object_name=object_name,
    #     ignore_orientation=True,
    # )
    # # move_demo(
    # #     simulated=SIMULATED,
    # #     world=world,
    # #     context=context,
    # #     target_pose="ROBOT_START_POSE",
    # # )


if __name__ == "__main__":
    main()
