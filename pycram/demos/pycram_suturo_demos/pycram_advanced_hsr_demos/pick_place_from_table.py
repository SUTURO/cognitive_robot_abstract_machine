from __future__ import annotations

import logging
import rclpy

from demos.pycram_suturo_demos.pycram_basic_hsr_demos.move_demo import move_demo
from demos.pycram_suturo_demos.pycram_basic_hsr_demos.pickup_demo_marc import (
    pickup_demo,
)
from demos.pycram_suturo_demos.pycram_basic_hsr_demos.place_demo import place_demo
from pycram.datastructures.pose import PoseStamped

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

from demos.pycram_suturo_demos.helper_methods_and_useful_classes.robot_setup import (
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
        from demos.pycram_suturo_demos.helper_methods_and_useful_classes.object_creation import (
            perceive_and_spawn_all_objects,
        )

        perceived_objects = perceive_and_spawn_all_objects(world=world)
    except ImportError:
        print("Could not import robokudo")
        perceived_objects = {}
    return perceived_objects


def main():
    rclpy.init()
    SIMULATED = True

    rclpy_node, world, robot_view, context = initialization(simulation=SIMULATED)
    objects = {1: "milk.stl", 2: "breakfast_cereal.stl"}

    perceived_objects = try_perceive_and_spawn(world)
    objects.update(perceived_objects)

    # put NLP-pipeline here
    object_name: str = input(
        f"Which object do you want to pick up? Currently known objects: {objects}"
    )

    object_to_pickup = world.get_body_by_name(object_name)

    # change the coords accordingly
    place_pose = PoseStamped.from_list(
        [1.9, 3.3, 0.7], [0, 0, 1, 0.1], frame=world.root
    )
    move_demo(
        simulated=SIMULATED, world=world, context=context, target_pose="POPCORN_TABLE"
    )
    pickup_demo(
        simulation=SIMULATED,
        context=context,
        object_to_pickup=object_to_pickup,
    )
    place_demo(
        simulation=SIMULATED,
        place_pose=place_pose,
        hsrb_world=world,
        context=context,
        object_name=object_name,
    )
    move_demo(
        simulated=SIMULATED,
        world=world,
        context=context,
        target_pose="ROBOT_START_POSE",
    )


if __name__ == "__main__":
    main()
