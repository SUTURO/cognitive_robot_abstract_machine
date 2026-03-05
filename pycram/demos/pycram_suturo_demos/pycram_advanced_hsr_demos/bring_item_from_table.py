from __future__ import annotations

import logging
from typing import Any

import rclpy

from demos.pycram_suturo_demos.pycram_basic_hsr_demos.move_demo import move_demo
from demos.pycram_suturo_demos.pycram_basic_hsr_demos.pickup_demo_marc import (
    pickup_demo,
)
from pycram.datastructures.dataclasses import Context
from semantic_digital_twin.robots.abstract_robot import Manipulator
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.world import World

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


def initialization(simulation: bool, with_simulated_objects: bool = False):
    result = robot_setup(
        simulation=simulation, with_simulated_objects=with_simulated_objects
    )

    return (
        result.node,
        result.world,
        result.robot_view,
        result.context,
        result.manipulator,
    )


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
    try:
        simulated = True
        with_simulated_objects = True

        rclpy_node, world, robot_view, context, manipulator = initialization(
            simulation=simulated, with_simulated_objects=with_simulated_objects
        )
        # objects = {1: "milk.stl", 2: "breakfast_cereal.stl"} if WITH_OBJECTS else {}
        #
        # perceived_objects = try_perceive_and_spawn(world)
        # objects.update(perceived_objects)

        # object_name: str = input(f"Which object do you want to pick up?")

        #
        # move_demo(
        #     simulated=simulated,
        #     world=world,
        #     context=context,
        #     target_pose="POPCORN_TABLE",
        # )
        cooking_table_annotation = world.get_semantic_annotation_by_name(
            "cooking_table"
        )
        print(cooking_table_annotation)
        print(robot_view.root)
        nearest = query_get_next_object_euclidean_x_y(
            robot_view.root, cooking_table_annotation
        ).tolist()
        on_table = query_semantic_annotations_on_surfaces(
            [cooking_table_annotation], world
        ).tolist()
        red_objects_on_table = query_annotations_by_color(Color.RED(), on_table)
        print(nearest[0])
        milkme_harder = world.get_body_by_name("milk.stl").visual.shapes[0].color
        print("color:", red_objects_on_table)
        # pickup_demo(
        #     simulation=simulated,
        #     hsrb_world=world,
        #     context=context,
        #     object_name=object_name,
        # )
        # move_demo(
        #     simulated=simulated,
        #     world=world,
        #     context=context,
        #     target_pose="ROBOT_START_POSE",
        # )
        world.clear()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
