import logging
import time

from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.robot_plans import (
    GiskardPickUpActionDescription,
    GiskardPlaceActionDescription,
)
from pycram_suturo_demos.helper_methods_and_useful_classes.A_robot_setup import (
    robot_setup,
)
from pycram.datastructures.enums import Arms
from pycram.motion_executor import real_robot, simulated_robot
from pycram_suturo_demos.pycram_basic_hsr_demos.move_demo import move_demo

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
object_to_pickup_startpose = PoseStamped.from_list(
    [
        object_to_pickup.global_pose.x,
        object_to_pickup.global_pose.y,
        object_to_pickup.global_pose.z,
    ],
    [0, 0, 0, 1],
    frame=world.root,
)


plan = SequentialPlan(context, ParkArmsActionDescription(Arms.BOTH))
plan_move = SequentialPlan(context, move_demo(simulated=SIMULATED,context=context,world=world, target_pose="POPCORN_TABLE"))
plan2 = SequentialPlan(
    context,
    GiskardPickUpActionDescription(
        simulated=SIMULATED,
        object_designator=object_to_pickup,
        arm=Arms.LEFT,
        gripper_vertical=True,
    ),
)
plan3 = SequentialPlan(
    context,
    GiskardPlaceActionDescription(
        simulated=SIMULATED,
        object_designator=object_to_pickup,
        arm=Arms.LEFT,
        target_location=object_to_pickup_startpose,
    ),
)
with robot_type:
    plan.perform()
    print("parked arms")
    plan2.perform()
    plan3.perform()
