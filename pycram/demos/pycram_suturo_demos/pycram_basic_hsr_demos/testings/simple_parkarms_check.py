import logging
import time

from pycram.language import SequentialPlan
from pycram.robot_plans import GiskardPickUpActionDescription, GiskardPlaceActionDescription
from pycram_suturo_demos.helper_methods_and_useful_classes.A_robot_setup import (
    robot_setup,
)
from pycram.datastructures.enums import Arms
from pycram.motion_executor import real_robot, simulated_robot

print("before ParkArms import")
from pycram.robot_plans.actions.core.robot_body import ParkArmsActionDescription

print("after ParkArms import")

logger = logging.getLogger(__name__)

SIMULATED = True
robot_type = simulated_robot if SIMULATED else real_robot

result = robot_setup(simulation=SIMULATED)
world, robot, context, node = (
    result.world,
    result.robot_view,
    result.context,
    result.node,
)
object_to_pickup = world.get_body_by_name("milk.stl")

plan = SequentialPlan(context, ParkArmsActionDescription(Arms.BOTH))
plan2 = SequentialPlan(
    context,
    GiskardPickUpActionDescription(simulated=SIMULATED,
        object_designator=object_to_pickup, arm=Arms.LEFT, gripper_vertical=True
    ),
)
print()
plan3 = SequentialPlan(
    context,
    GiskardPlaceActionDescription(simulated=SIMULATED,
    object_designator=object_to_pickup, arm=Arms.LEFT, target_location=object_to_pickup.global_pose.to_pose().to
    ),
)
with robot_type:
    plan.perform()
    print("parked arms")
    plan2.perform()
    plan3.perform()
