import rclpy
from rclpy.logging import get_logger

import semantic_digital_twin.exceptions
from giskardpy.executor import Executor, SimulationPacer
from giskardpy.motion_statechart.goals.pick_up import PickUp
from giskardpy.motion_statechart.graph_node import EndMotion
from giskardpy.motion_statechart.monitors.payload_monitors import CountSeconds
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy.qp.qp_controller_config import QPControllerConfig
from pycram.datastructures.enums import Arms, VerticalAlignment, ApproachDirection
from pycram.datastructures.grasp import GraspDescription
from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot, simulated_robot, ExecutionEnvironment
from pycram.robot_plans import (
    ParkArmsActionDescription,
    PickUpAction,
    PickUpActionDescription,
    PickupMotion,
)
from demos.pycram_suturo_demos.helper_methods_and_useful_classes.robot_setup import (
    robot_setup,
)
from semantic_digital_twin.adapters.ros import (
    HomogeneousTransformationMatrixToRos2Converter,
)
from semantic_digital_twin.robots.abstract_robot import Manipulator, ParallelGripper
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body

# ------------------------ BASE-DEFINITIONS
logger = get_logger(__name__)

SIMULATED: bool = True
with_perception: bool = False
object_name: str = ""

robot_with_perception: bool = not SIMULATED and with_perception
robot_with_object_search: bool = not object_name == ""

robot_type: ExecutionEnvironment = simulated_robot if SIMULATED else real_robot

result = robot_setup(SIMULATED)

manipulator: ParallelGripper
hsrb_world, robot_view, context, manipulator = (
    result.world,
    result.robot_view,
    result.context,
    result.manipulator,
)

grasp: GraspDescription = GraspDescription(
    approach_direction=ApproachDirection.FRONT,
    vertical_alignment=VerticalAlignment.NoAlignment,
    manipulator=manipulator,
    rotate_gripper=False,
)


def try_get_object_to_pickup(world, object_name_method) -> Body | None:
    try:
        object_to_pickup_method = hsrb_world.get_body_by_name(object_name_method)
        logger.info(f"picking up object with name '{object_name_method}'")
    except semantic_digital_twin.exceptions.WorldEntityNotFoundError:
        object_to_pickup_method = None
        logger.error(f"No object with name '{object_name_method}' found")
    return object_to_pickup_method


# -------------------------------- DETERMIN OBJECT_TO_PICKUP
if SIMULATED:
    object_name: str = "milk.stl"
    object_to_pickup = try_get_object_to_pickup(hsrb_world, object_name)
else:
    # navigate to location
    # spawn percieved objects
    # search and pickup
    object_to_pickup = try_get_object_to_pickup(hsrb_world, object_name)

# -------------------------------- PLANNING
if object_to_pickup is not None:
    plan = SequentialPlan(
        context,
        ParkArmsActionDescription(Arms.BOTH),
        PickUpActionDescription(
            object_designator=object_to_pickup, arm=Arms.LEFT, grasp_description=grasp
        ),
        ParkArmsActionDescription(Arms.BOTH),
    )
else:
    plan = SequentialPlan(
        context,
        ParkArmsActionDescription(Arms.BOTH),
    )

# ------------------------ EXECUTION
with simulated_robot:
    plan.perform()

rclpy.shutdown()
