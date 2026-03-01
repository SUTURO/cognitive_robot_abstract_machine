from typing import Any

import rclpy
from rclpy.logging import get_logger

import semantic_digital_twin.exceptions

from pycram.datastructures.enums import Arms
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot, simulated_robot, ExecutionEnvironment
from pycram.robot_plans import (
    ParkArmsActionDescription,
    GiskardPickUpActionDescription,
)
from demos.pycram_suturo_demos.helper_methods_and_useful_classes.robot_setup import (
    robot_setup,
)
from semantic_digital_twin.robots.abstract_robot import ParallelGripper

from demos.pycram_suturo_demos.helper_methods_and_useful_classes.pickup_helper_methods import (
    attach_object_to_hsrb,
    try_get_object_to_pickup,
)

# ------------------------ BASE-DEFINITIONS
logger = get_logger(__name__)

SIMULATED: bool = True
with_perception: bool = False
object_name: str = ""

robot_type: ExecutionEnvironment = simulated_robot if SIMULATED else real_robot


result = robot_setup(simulation=SIMULATED)

hsrb_world, robot_view, context, node = (
    result.world,
    result.robot_view,
    result.context,
    result.node,
)

# IMPORTANT: giskardpy's ROS2 ActionClient needs a valid rclpy.node.Node.
# PyCRAM passes context.ros_node down into the MotionExecutor.
if getattr(context, "ros_node", None) is None:
    context.ros_node = node
# Some setups name it differently; set it too if present.
if hasattr(context, "node") and getattr(context, "node", None) is None:
    context.node = node

manipulator = hsrb_world.get_semantic_annotations_by_type(ParallelGripper)[0]
# -------------------------------- DETERMIN OBJECT_TO_PICKUP
if SIMULATED:
    object_name = "milk.stl"
    object_to_pickup = try_get_object_to_pickup(hsrb_world, object_name)
else:
    if with_perception:
        from demos.pycram_suturo_demos.helper_methods_and_useful_classes.object_creation import (
            perceive_and_spawn_all_objects,
        )

        perceived_objects: dict[Any, Any] = perceive_and_spawn_all_objects(hsrb_world)
        logger.info(f"perceived following objects: '{perceived_objects}'")
    object_to_pickup = try_get_object_to_pickup(hsrb_world, object_name)
    logger.info(f"object_to_Pickup: '{object_to_pickup}'")

# -------------------------------- PLANNING
plan = SequentialPlan(
    context,
    GiskardPickUpActionDescription(
        object_designator=object_to_pickup, arm=Arms.LEFT, gripper_vertical=True
    ),
)

plan2 = SequentialPlan(context, ParkArmsActionDescription(Arms.BOTH))
# ------------------------ EXECUTION
try:
    with simulated_robot:
        plan.perform()
        attach_object_to_hsrb(
            world=hsrb_world,
            object_designator=object_to_pickup,
            manipulator=manipulator,
        )
        plan2.perform()
finally:
    # Always shut down cleanly even if planning/execution raises.
    if rclpy.ok():
        rclpy.shutdown()
