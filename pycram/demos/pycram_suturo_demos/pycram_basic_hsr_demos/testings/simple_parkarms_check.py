import logging
import time

from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.robot_plans import (
    GiskardPickUpActionDescription,
    GiskardPlaceAndDetachActionDescription,
)
from pycram_suturo_demos.helper_methods_and_useful_classes.A_robot_setup import (
    robot_setup,
)
from pycram.datastructures.enums import Arms
from pycram.motion_executor import real_robot, simulated_robot
from pycram_suturo_demos.helper_methods_and_useful_classes.pickup_helper_methods import (
    look_at_point,
)
from pycram_suturo_demos.pycram_advanced_hsr_demos.bring_object_from_table_to_shelf_demo import (
    try_and_scan_for_object_on_table,
)
from pycram_suturo_demos.pycram_basic_hsr_demos.move_demo import move_demo
from semantic_digital_twin.semantic_annotations.semantic_annotations import Milk
from semantic_digital_twin.spatial_types import Point3, HomogeneousTransformationMatrix

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


table = world.get_semantic_annotation_by_name("cooking_table")
look_at = table.global_pose.to_position()

plan = SequentialPlan(context, ParkArmsActionDescription(Arms.BOTH))
plan_move = move_demo(
    simulated=SIMULATED, context=context, world=world, target_pose="POPCORN_TABLE"
)
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
    GiskardPlaceAndDetachActionDescription(
        simulated=SIMULATED,
        object_designator=object_to_pickup,
        arm=Arms.LEFT,
        target_location=object_to_pickup_startpose,
        ignore_orientation=True,
    ),
)

with robot_type:
    look_at_point(context, look_at)
    plan.perform()
    print("parked arms")
    plan2.perform()
    look_at_point(context, look_at)
    plan3.perform()
