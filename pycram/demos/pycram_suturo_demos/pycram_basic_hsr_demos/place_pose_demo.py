import logging

import rclpy

import semantic_digital_twin
from demos.pycram_suturo_demos.helper_methods_and_useful_classes.object_creation import (
    spawn_semantic_with_body,
)
from demos.pycram_suturo_demos.helper_methods_and_useful_classes.place_pose import (
    get_pose_on_semantic_annotation_for_object_by_semantic_annotation,
)
from demos.pycram_suturo_demos.helper_methods_and_useful_classes.robot_setup import (
    robot_setup,
)
from pycram.motion_executor import simulated_robot, real_robot
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Milk,
)
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.geometry import Scale

SIMULATED = True
"""
Set this flag to True to run the demo in a simulated environment, 
or False to run it on the real robot.
"""

logging.getLogger(semantic_digital_twin.world.__name__).setLevel(logging.WARN)
logger = logging.getLogger(__name__)


def simulation_demo():
    rclpy.init()
    setup_result = robot_setup(simulation=True, with_simulated_objects=False)
    world, robot_view, context = (
        setup_result.world,
        setup_result.robot_view,
        setup_result.context,
    )

    # Spawn example object
    with world.modify_world():
        milk = Milk.create_with_new_body_in_world(
            name=PrefixedName("milk_carton"), world=world, scale=Scale(0.1, 0.1, 0.2)
        )

    # Get pose
    pose = get_pose_on_semantic_annotation_for_object_by_semantic_annotation(
        "desk", milk, world
    )

    # "Error handling"
    pose = pose if pose is not None else Pose()

    # Spawn object at newfound location
    milk = spawn_semantic_with_body(
        "Milk",
        "milk_carton",
        Scale(0.1, 0.1, 0.2),
        pose,
        world,
    )


def real_demo():
    # TODO: Implement
    pass


if __name__ == "__main__":
    if SIMULATED:
        with simulated_robot:
            simulation_demo()
    else:
        with real_robot:
            real_demo()
