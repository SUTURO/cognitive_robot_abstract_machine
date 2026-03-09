from __future__ import annotations

import logging

from demos.pycram_suturo_demos.helper_methods_and_useful_classes.pickup_helper_methods import parse_color, \
    try_perceive_and_spawn, initialization, object_to_pickup_by_mode, get_pickup_mode
from demos.pycram_suturo_demos.pycram_basic_hsr_demos.move_demo import move_demo
from demos.pycram_suturo_demos.pycram_basic_hsr_demos.pickup_demo_marc import (
    pickup_demo
)

logger = logging.getLogger(__name__)

def main():
    simulated = True
    with_simulated_objects = True

    rclpy_node, world, robot_view, context, manipulator = initialization(
        simulation=simulated, with_simulated_objects=with_simulated_objects
    )

    pickup_mode, object_name, object_color = get_pickup_mode()

    move_demo(
        simulated=simulated,
        world=world,
        context=context,
        target_pose="POPCORN_TABLE",
    )

    try_perceive_and_spawn(world)
    object_to_pickup = object_to_pickup_by_mode(world=world, mode=pickup_mode, object_name=object_name, color=object_color)

    pickup_demo(
        simulation=simulated,
        context=context,
        object_to_pickup=object_to_pickup,
    )
    move_demo(
        simulated=simulated,
        world=world,
        context=context,
        target_pose="ROBOT_START_POSE",
    )

if __name__ == "__main__":
    main()
