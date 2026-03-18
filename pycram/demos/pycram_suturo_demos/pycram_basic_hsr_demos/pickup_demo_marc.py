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
)
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body


# ------------------------ BASE-DEFINITIONS
def pickup_demo(
    simulation: bool = True,
    context: Context = None,
    object_to_pickup: Body = None,
):
    # logger creaton
    logger = logging.get_logger(__name__)

    # if the determined object is None, the pickup is skipped, because the object was not parsed properly
    if object_to_pickup == None:
        logger.warning("object_to_pickup is None, therefor pickup is skipped")
        return

    robot_type: ExecutionEnvironment = simulated_robot if simulation else real_robot

    # -------------------------------- PLANNING

    plan_pullup = SequentialPlan(
        context,
        GiskardPullUpActionDescription(
            arm=Arms.LEFT, object_designator=object_to_pickup, simulated=simulation
        ),
        ParkArmsActionDescription(Arms.BOTH),
        MoveTorsoActionDescription(TorsoState.LOW),
    )

    # ------------------------ EXECUTION
    with robot_type:
        logger.info("Starting pickup demo")
        SequentialPlan(
            context,
            GiskardGraspActionDescription(
                simulated=simulation,
                object_designator=object_to_pickup,
                arm=Arms.LEFT,
                gripper_vertical=True,
            ),
        ).perform()
        while input("Was the object grasped? ").strip().lower() != "yes":
            # retract and regrasp
            SequentialPlan(
                context,
                GiskardRetractActionDescription(simulated=simulation, arm=Arms.LEFT),
                ParkArmsActionDescription(Arms.BOTH),
                GiskardGraspActionDescription(
                    simulated=simulation,
                    object_designator=object_to_pickup,
                    arm=Arms.LEFT,
                    gripper_vertical=True,
                ),
            ).perform()
        plan_pullup.perform()
        logger.info("parking arms finished")
        logger.info("PickUp has been executed")
