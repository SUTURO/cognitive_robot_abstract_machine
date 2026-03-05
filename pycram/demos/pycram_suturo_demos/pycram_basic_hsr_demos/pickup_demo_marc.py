from typing import Any

import rclpy
from rclpy import logging
from rclpy.logging import get_logger

from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot, simulated_robot, ExecutionEnvironment
from pycram.robot_plans import (
    ParkArmsActionDescription,
    GiskardPickUpActionDescription, MoveGripperMotion, GiskardMoveGripperMotion, GiskardRetractActionDescription,
    GiskardPlaceActionDescription,
)
from demos.pycram_suturo_demos.helper_methods_and_useful_classes.pickup_helper_methods import (
    attach_object_to_hsrb,
    try_perceiving_and_spawning_and_find_object,
)
from semantic_digital_twin.world import World
from test.pycram_test.ontomatic.ontomatic_performable_test_dummy import GripperState


# ------------------------ BASE-DEFINITIONS
def pickup_demo(
    simulation: bool = True,
    hsrb_world: World = None,
    context: Context = None,
    object_name: str = "",
):
    logger = logging.get_logger(__name__)

    SIMULATED: bool = simulation
    object_name: str = object_name

    robot_type: ExecutionEnvironment = simulated_robot if SIMULATED else real_robot

    # -------------------------------- DETERMIN OBJECT_TO_PICKUP
    object_to_pickup = try_perceiving_and_spawning_and_find_object(
        world=hsrb_world, object_name=object_name
    )

    logger.info(f"object_to_pickup: {object_to_pickup}")

    # -------------------------------- PLANNING
    plan = SequentialPlan(
            context,
            GiskardPickUpActionDescription(
                simulated=simulation,
                object_designator=object_to_pickup,
                arm=Arms.LEFT,
                gripper_vertical=True,
            ),
        )

    plan_park = SequentialPlan(context, ParkArmsActionDescription(Arms.BOTH))

    # ------------------------ EXECUTION
    with robot_type:
        logger.info("Starting pickup demo")
        plan.perform()
        logger.info("pickup finished")
        verification: str = input("Is the object in the gripper? (yes/no)")
        while verification != "yes":
            # TODO opengripper and set back from current position / retracting
            SequentialPlan(context,
                           GiskardPlaceActionDescription(
                               simulated=simulation,
                               object_designator=object_to_pickup,
                               arm=Arms.LEFT,
                               target_location=PoseStamped.from_spatial_type(object_to_pickup.global_pose))).perform()
            SequentialPlan(
                context,
                GiskardMoveGripperMotion(
                    simulated=simulation,
                    motion=GripperState.OPEN,
                    gripper=Arms.LEFT,
                    allow_gripper_collision=False,
                ),
            ).perform()

            SequentialPlan(
                context,
                GiskardPickUpActionDescription(
                    simulated=simulation,
                    object_designator=object_to_pickup,
                    arm=Arms.LEFT,
                    gripper_vertical=True,
                ),
            )
            verification: str = input("Is the object in the gripper? (yes/no)")
        attach_object_to_hsrb(world=hsrb_world, object_designator=object_to_pickup)
        logger.info("parking arms")
        plan_park.perform()
        logger.info("parking arms finished")
