import time

from rclpy import logging

from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms, PickUpType
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot, simulated_robot, ExecutionEnvironment
from pycram.robot_plans import (
    ParkArmsActionDescription,
    GiskardRetractActionDescription,
    GiskardGraspActionDescription,
    GiskardPullUpActionDescription,
    MoveTorsoActionDescription,
    GiskardPickUpActionDescription,
)
from pycram_suturo_demos.helper_methods_and_useful_classes.nlp_human_robot_interaction import (
    TalkingNode,
)
from pycram_suturo_demos.helper_methods_and_useful_classes.pickup_helper_methods import (
    validate_grasped,
)

from semantic_digital_twin.world_description.world_entity import Body


# ------------------------ BASE-DEFINITIONS
def pickup_demo(
    simulation: bool = True,
    context: Context = None,
    object_to_pickup: Body = None,
):
    # logger creaton
    talking_node = TalkingNode()
    standard_delay = 2
    logger = logging.get_logger(__name__)

    # if the determined object is None, the pickup is skipped, because the object was not parsed properly
    if object_to_pickup == None:
        logger.warning("object_to_pickup is None, therefor pickup is skipped")
        return

    robot_type: ExecutionEnvironment = simulated_robot if simulation else real_robot

    # ------------------------ EXECUTION
    with robot_type:
        talking_node.pub(text="Stating pickup", delay=standard_delay)
        logger.info("Starting pickup demo")
        pickup_callback = SequentialPlan(
            context,
            GiskardPickUpActionDescription(
                simulated=simulation,
                object_designator=object_to_pickup,
                arm=Arms.LEFT,
                gripper_vertical=True,
            ),
        ).perform()

        logger.info("PickUp has is now finished", delay=standard_delay)


def pickup_with_callback_demo(
    simulation: bool = True,
    context: Context = None,
    object_to_pickup: Body = None,
):
    # logger creaton
    talking_node = TalkingNode()
    standard_delay = 2
    logger = logging.get_logger(__name__)

    # if the determined object is None, the pickup is skipped, because the object was not parsed properly
    if object_to_pickup == None:
        logger.warning("object_to_pickup is None, therefor pickup is skipped")
        return

    robot_type: ExecutionEnvironment = simulated_robot if simulation else real_robot

    # ------------------------ EXECUTION
    with robot_type:
        talking_node.pub(text="Stating pickup", delay=standard_delay)
        logger.info("Starting pickup demo")
        pickup_callback = SequentialPlan(
            context,
            GiskardPickUpActionDescription(
                simulated=simulation,
                object_designator=object_to_pickup,
                arm=Arms.LEFT,
                gripper_vertical=True,
            ),
        ).perform()
        logger.info("PickUp has is now finished", delay=standard_delay)
        return pickup_callback
