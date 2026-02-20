from helper_methods_and_useful_classes import real_setup
from helper_methods_and_useful_classes.simulation_setup import setup_hsrb_in_environment
from suturo_resources.suturo_map import load_environment


def robot_setup(simulation: bool = True, with_objects: bool = True):
    return setup_hsrb_in_environment(load_environment=load_environment(), with_viz=True, with_obj=with_objects
                                     ) if simulation else real_setup.world_setup_with_test_objects()