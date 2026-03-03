from demos.pycram_suturo_demos.helper_methods_and_useful_classes.object_creation import (
    spawn_semantic_with_body,
)
from demos.pycram_suturo_demos.helper_methods_and_useful_classes.robot_setup import (
    robot_setup,
)
from pycram.motion_executor import simulated_robot, real_robot
from semantic_digital_twin.semantic_annotations.semantic_annotations import Table
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.geometry import Scale

SIMULATED = True
"""
Set this flag to True to run the demo in a simulated environment, 
or False to run it on the real robot.
"""


def simulation_demo():
    setup_result = robot_setup(
        simulation=True, with_objects=True, with_perception=False
    )
    world, robot_view, context = (
        setup_result.world,
        setup_result.robot_view,
        setup_result.context,
    )

    # Get semantic annotation of object you want to sample points from
    table: Table = world.get_semantic_annotations_by_name("desk_annotation")[0]

    # Example object to sample for
    milk = spawn_semantic_with_body(
        "Milk", "milk_carton", Scale(0.1, 0.1, 0.2), Pose(), world
    )

    # Sample points from surface and get the first one
    points = table.sample_points_from_surface(milk)
    point = points[0]

    # Spawn object at newfound location
    milk = spawn_semantic_with_body(
        "Milk",
        "milk_carton",
        Scale(0.1, 0.1, 0.2),
        Pose(position=point, reference_frame=point.reference_frame),
        world,
    )


def real_demo():
    setup_result = robot_setup(
        simulation=False, with_objects=True, with_perception=False
    )
    world, robot_view, context = (
        setup_result.world,
        setup_result.robot_view,
        setup_result.context,
    )

    # TODO: Implement
    pass


if __name__ == "__main__":
    if SIMULATED:
        with simulated_robot:
            simulation_demo()
    else:
        with real_robot:
            real_demo()
