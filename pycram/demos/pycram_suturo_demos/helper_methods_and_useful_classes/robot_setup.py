from dataclasses import field
from typing import Tuple, Optional

from suturo_resources.suturo_map import load_environment


def robot_setup(
    simulation: bool = field(default=True, kw_only=True),
    with_objects: bool = field(default=True, kw_only=True),
):
    """
    Method that calls either simulation or real, based on the bool

    :param bool simulation: True or False, based on if SIMULATED or REAL
    :param bool with_objects: True or False, based on if you want to spawn basic objects or not
    :param bool with_perception: True or False, if you want to percieve in REAL
    :param Tuple[float, float, float, float, float, float] robot_xyz_rpy: xyz and rpy of the robot
    """
    if simulation:
        from demos.pycram_suturo_demos.helper_methods_and_useful_classes.simulation_setup import (
            setup_hsrb_in_environment,
        )

        setup_result = setup_hsrb_in_environment(
            load_environment=load_environment,
            with_objects=with_objects,
        )

    else:
        from demos.pycram_suturo_demos.helper_methods_and_useful_classes.real_setup import (
            world_setup_with_test_objects,
        )

        setup_result = world_setup_with_test_objects(with_object=with_objects)

    return setup_result


load_environment
