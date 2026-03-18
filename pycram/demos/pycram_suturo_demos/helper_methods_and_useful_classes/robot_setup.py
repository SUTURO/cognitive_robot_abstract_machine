from suturo_resources.suturo_map import load_environment


def robot_setup(
    *,
    simulation: bool = True,
    with_simulated_objects: bool = True,
):
    """
    Set up the robot either in simulation or on the real system.

    Args:
        simulation: If True, run in simulation; otherwise use the real robot.
        with_simulated_objects: If True, spawn basic objects in the world.

    Returns:
        Setup result object from the respective setup routine.
    """
    if simulation:
        from pycram_suturo_demos.helper_methods_and_useful_classes.simulation_setup import (
            setup_hsrb_in_environment,
        )

        return setup_hsrb_in_environment(
            load_environment=load_environment,
            with_simulated_objects=with_simulated_objects,
        )

    from demos.pycram_suturo_demos.helper_methods_and_useful_classes.real_setup import (
        world_setup_with_test_objects,
    )

    return world_setup_with_test_objects(with_object=with_simulated_objects)
