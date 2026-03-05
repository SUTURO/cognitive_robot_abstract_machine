from __future__ import annotations

import logging
from typing import List, Dict

import rclpy
from suturo_resources.queries import query_semantic_annotations_on_surfaces
from sympy import false

import semantic_digital_twin
from demos.pycram_suturo_demos.helper_methods_and_useful_classes import (
    place_pose,
    object_creation,
)
from demos.pycram_suturo_demos.helper_methods_and_useful_classes.robot_setup import (
    robot_setup,
)
from demos.pycram_suturo_demos.pycram_basic_hsr_demos.move_demo import move_demo
from demos.pycram_suturo_demos.pycram_basic_hsr_demos.pickup_demo_marc import (
    pickup_demo,
)
from demos.pycram_suturo_demos.pycram_basic_hsr_demos.place_demo import place_demo
from pycram.datastructures.pose import PoseStamped
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.semantic_annotations.mixins import (
    HasStorageSpace,
    HasRootBody,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Cucumber,
    Banana,
    Sauce,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Scale
from semantic_digital_twin.world_description.world_entity import SemanticAnnotation

logger = logging.getLogger(__name__)
logging.getLogger(semantic_digital_twin.world.__name__).setLevel(logging.WARN)


def initialization(simulation: bool = True):
    result = robot_setup(simulation=simulation, with_simulated_objects=false)
    rclpy_node, world, robot_view, context = (
        result.node,
        result.world,
        result.robot_view,
        result.context,
    )
    return rclpy_node, world, robot_view, context


def spawn_ref_objects(world: World):
    scale = Scale(0.1, 0.1, 0.2)
    with world.modify_world():
        cucumber = Cucumber.create_with_new_body_in_world(
            name=PrefixedName("ref_cucumber"), world=world, scale=scale
        )
        banana = Banana.create_with_new_body_in_world(
            name=PrefixedName("ref_banana"), world=world, scale=scale
        )

    pose = place_pose.get_pose_on_semantic_annotation_for_object_by_semantic_annotation(
        "dining_table", cucumber, world
    )
    with world.modify_world():
        object_creation.spawn_semantic_with_body(
            cucumber, str(cucumber.name.name), scale, pose, world
        )

    pose = place_pose.get_pose_on_semantic_annotation_for_object_by_semantic_annotation(
        "table", banana, world
    )
    with world.modify_world():
        object_creation.spawn_semantic_with_body(
            banana, str(banana.name.name), scale, pose, world
        )


def spawn_objects_to_pick(world: World):
    scale = Scale(0.1, 0.1, 0.2)
    objects_to_pick: List[HasRootBody] = []
    with world.modify_world():
        cucumber = Cucumber.create_with_new_body_in_world(
            name=PrefixedName("cucumber"), world=world, scale=scale
        )
        objects_to_pick.append(cucumber)
        sauce = Sauce.create_with_new_body_in_world(
            name=PrefixedName("sauce"), world=world, scale=scale
        )
        objects_to_pick.append(sauce)
        banana = Banana.create_with_new_body_in_world(
            name=PrefixedName("banana"), world=world, scale=scale
        )
        objects_to_pick.append(banana)

    cooking_table: SemanticAnnotation = world.get_semantic_annotation_by_name(
        "cooking_table"
    )
    if not isinstance(cooking_table, HasStorageSpace):
        logger.warning(
            f'Object with name "{cooking_table.name}" has no root body. Cannot sample points for object.'
        )
        return

    for obj in objects_to_pick:
        print(obj.name.name)
        pose = place_pose.get_pose_on_semantic_annotation_for_object_by_semantic_annotation(
            cooking_table, obj, world
        )
        with world.modify_world():
            object_creation.spawn_semantic_with_body(
                obj, str(obj.name.name), scale, pose, world
            )


def main():
    rclpy.init()
    SIMULATED = True

    rclpy_node, world, robot_view, context = initialization(simulation=SIMULATED)

    spawn_objects_to_pick(world)
    spawn_ref_objects(world)

    table = world.get_semantic_annotation_by_name("cooking_table")
    objs = query_semantic_annotations_on_surfaces([table], world)
    print(objs)
    return

    # put NLP-pipeline here
    object_name = "milk.stl"
    place_pose = PoseStamped.from_list(
        [1.9, 3.3, 0.7], [0, 0, 1, 0.1], frame=world.root
    )
    move_demo(
        world=world, context=context, target_pose="POPCORN_TABLE", simulated=SIMULATED
    )
    pickup_demo(
        simulation=SIMULATED,
        hsrb_world=world,
        context=context,
        object_name=object_name,
    )
    place_demo(
        place_pose=place_pose,
        hsrb_world=world,
        context=context,
        object_name=object_name,
        simulation=SIMULATED,
    )
    move_demo(
        world=world,
        context=context,
        target_pose="ROBOT_START_POSE",
        simulated=SIMULATED,
    )


if __name__ == "__main__":
    main()
