import logging

import semantic_digital_twin
from demos.pycram_suturo_demos.helper_methods_and_useful_classes.object_creation import (
    spawn_semantic_with_body,
    perceive_and_spawn_all_objects,
)
from demos.pycram_suturo_demos.helper_methods_and_useful_classes.robot_setup import (
    robot_setup,
)
from pycram.motion_executor import simulated_robot, real_robot
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Scale

logger = logging.getLogger(__name__)
logging.getLogger(semantic_digital_twin.world.__name__).setLevel(logging.WARN)


SIMULATED = True
"""
Set this flag to True to run the demo in a simulated environment, 
or False to run it on the real robot.
"""


def simulation_demo():
    """
    Demonstrates object creation via the method used for spawning perceived objects in a simulated environment.
    """

    setup_result = robot_setup(simulation=True, with_simulated_objects=True)
    world, robot_view, context = (
        setup_result.world,
        setup_result.robot_view,
        setup_result.context,
    )

    # Spawning an object with example data from perception.
    spawn_semantic_with_body(
        "Spoon",
        "spoon_dinner_redgrip",
        Scale(0.206, 0.052, 0.0287),
        Pose.from_xyz_quaternion(1.427, 4.993, 0.723, 0.0, 0.0, 0.998, 0.054),
        world,
    )


def real_demo():
    """
    Runs perception on the real robot and spawns all perceived objects in the world.
    """

    setup_result = robot_setup(
        simulation=False, with_simulated_objects=True, with_perception=False
    )
    world, robot_view, context = (
        setup_result.world,
        setup_result.robot_view,
        setup_result.context,
    )
    perceive_and_spawn_all_objects(world=world)


if __name__ == "__main__":
    if SIMULATED:
        with simulated_robot:
            simulation_demo()
    else:
        with real_robot:
            real_demo()
