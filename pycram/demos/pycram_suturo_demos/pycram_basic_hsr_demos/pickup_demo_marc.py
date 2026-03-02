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
    GiskardPickUpActionDescription,
    PlaceActionDescription,
    GiskardPlaceActionDescription,
)
from demos.pycram_suturo_demos.helper_methods_and_useful_classes.robot_setup import (
    robot_setup,
)

from demos.pycram_suturo_demos.helper_methods_and_useful_classes.pickup_helper_methods import (
    attach_object_to_hsrb,
    try_get_object_to_pickup,
    detach_object_from_hsrb,
)
from pycram.ros import sleep
from semantic_digital_twin.world import World


# ------------------------ BASE-DEFINITIONS
def pickup_demo(
    simulation: bool = True,
    hsrb_world: World = None,
    context: Context = None,
    with_perception: bool = False,
    object_name: str = "",
):
    logger = logging.get_logger(__name__)

    SIMULATED: bool = simulation
    with_perception: bool = with_perception
    object_name: str = object_name

    robot_type: ExecutionEnvironment = simulated_robot if SIMULATED else real_robot
    #
    # result = robot_setup(simulation=SIMULATED)
    #
    # hsrb_world, robot_view, context, node, manipulator = (
    #     result.world,
    #     result.robot_view,
    #     result.context,
    #     result.node,
    #     result.manipulator,
    # )
    #
    # # IMPORTANT: giskardpy's ROS2 ActionClient needs a valid rclpy.node.Node.
    # # PyCRAM passes context.ros_node down into the MotionExecutor.
    # if getattr(context, "ros_node", None) is None:
    #     context.ros_node = node
    # # Some setups name it differently; set it too if present.
    # if hasattr(context, "node") and getattr(context, "node", None) is None:
    #     context.node = node

    # -------------------------------- DETERMIN OBJECT_TO_PICKUP

    try:
        from demos.pycram_suturo_demos.helper_methods_and_useful_classes.object_creation import (
            perceive_and_spawn_all_objects,
        )

        perceived_objects: dict[Any, Any] = perceive_and_spawn_all_objects(hsrb_world)
        logger.info(f"perceived following objects: '{perceived_objects}'")
    except ImportError:
        logger.info("Could not import robokudo")
        perceived_objects = {}
    object_to_pickup = try_get_object_to_pickup(hsrb_world, object_name)
    logger.info(f"object_to_Pickup: '{object_to_pickup}'")

    place_pose = PoseStamped.from_list([0, 0, 0], [0, 0, 1, 0.1], frame=hsrb_world.root)

    # -------------------------------- PLANNING
    plan = SequentialPlan(
        context,
        GiskardPickUpActionDescription(
            object_designator=object_to_pickup, arm=Arms.LEFT, gripper_vertical=True
        ),
    )
    plan_park = SequentialPlan(context, ParkArmsActionDescription(Arms.BOTH))

    # ------------------------ EXECUTION
    try:
        with simulated_robot:
            plan.perform()
            attach_object_to_hsrb(
                hsrb_world=hsrb_world,
                object_designator=object_to_pickup,
            )
            plan_park.perform()

    except Exception as e:
        print(e)
