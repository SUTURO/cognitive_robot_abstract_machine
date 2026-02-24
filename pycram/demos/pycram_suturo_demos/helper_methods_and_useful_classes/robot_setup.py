from suturo_resources.suturo_map import load_environment

"""
Method that calls either simulation or real, based on the bool

:param bool simulation: True or False, based on if SIMULATED or REAL
:param bool with_objects: True or False, based on if you want to spawn basic objects or not
:param bool with_perception: True or False, if you want to percieve in REAL
"""


def robot_setup(
    simulation: bool = True, with_objects: bool = True, with_perception: bool = False
):
    if simulation:
        from demos.pycram_suturo_demos.helper_methods_and_useful_classes.simulation_setup import (
            setup_hsrb_in_environment,
        )

        return setup_hsrb_in_environment(
            load_environment=load_environment, with_viz=True, with_obj=with_objects
        )
    else:
        from demos.pycram_suturo_demos.helper_methods_and_useful_classes.real_setup import (
            world_setup_with_test_objects,
        )

        return world_setup_with_test_objects(
            with_object=with_objects, with_perception=with_perception
        )
