from __future__ import annotations

import logging
import time
from typing import Optional

import rclpy

from pycram.datastructures.dataclasses import Context
from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.robot_plans import (
    MoveTorsoAction,
    MoveTorsoActionDescription,
    LookAtActionDescription,
)
from pycram_suturo_demos.helper_methods_and_useful_classes.pickup_helper_methods import (
    initialization,
    object_to_pickup_by_mode,
    get_pickup_mode,
    perceive_and_spawn_all_objects, look_at_point, try_percieve_and_retrieve,
)
from pycram_suturo_demos.pycram_basic_hsr_demos.move_demo import move_demo
from pycram_suturo_demos.pycram_basic_hsr_demos.pickup_demo import (
    pickup_demo,
)
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.spatial_types import Point3
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body

from demos.pycram_suturo_demos.helper_methods_and_useful_classes.nlp_human_robot_interaction import (
    TalkingNode,
)

logger = logging.getLogger(__name__)


"""
Full sequence constains: 
1. Move to table
2. perceive from pratical angle 1, if object not found continue with up to 3 poses
3. pickup
4. verify pickup
5. move back to start

:param object_name:object_name, parsed by the NLP-Interface
:param object_color:object_color, parsed by the NLP-Interface
"""


def pickup_main(
    context: Context,
    object_name: str,
):
    rclpy.init()
    simulated = False
    table = ""

    talking_node = TalkingNode()
    standard_delay = 2

    world = context.world
    robot_view = context.robot

    table = world.get_body_by_name("cooking_table")


    # Move to table, on which the object is to be expected.
    move_to_table = move_demo(
        simulated=simulated,
        world=world,
        context=context,
        target_pose=table,
    )
    move_to_starting_pose = move_demo(
        simulated=simulated,
        world=world,
        context=context,
        target_pose="ROBOT_START_POSE",
    )

    talking_node.pub(text="Moving to table.", delay=standard_delay)
    move_to_table()

    # three attempts to perceive the object, and retrieve it from the world.
    for i in range(3):
        object_to_pickup = try_percieve_and_retrieve(world=world)
        if object_to_pickup is not None:
            break

    perceive_and_spawn_all_objects(world)
    object_to_pickup = world.get_body_by_name(object_name)

    # move to perception angle 1
    if object_to_pickup == None:
        talking_node.pub("I could not find the object. Going back to starting pose.")
        move_to_starting_pose()
        return

    talking_node.pub(
        text=f"Initializing pickup of {object_to_pickup.name}", delay=standard_delay
    )
    pickup_demo(
        simulation=simulated,
        context=context,
        object_to_pickup=object_to_pickup,
    )
    talking_node.pub(
        text=f"Pickup has been finished, now moving back to start position",
        delay=standard_delay,
    )
    move_to_starting_pose()
