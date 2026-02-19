import rclpy

from suturo_resources.suturo_map import load_environment

from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import PoseStamped
from pycram_suturo_demo.setup_real_robot import world_setup_with_test_objects
from semantic_digital_twin.datastructures.definitions import TorsoState
from pycram.language import SequentialPlan
from pycram.motion_executor import simulated_robot, real_robot
from pycram.robot_plans import (
    MoveTorsoActionDescription,
    ParkArmsActionDescription,
    LookAtActionDescription,
)
from simulation_setup import setup_hsrb_in_environment

rclpy.init()
result = world_setup_with_test_objects()
world, robot_view, context = (
    result.world,
    result.robot_view,
    result.context,
)

plan = SequentialPlan(
    context,
    ParkArmsActionDescription(Arms.BOTH),
    LookAtActionDescription([PoseStamped.from_list([2, 3.5, 0.75], frame=world.root)]),
)

with real_robot:
    plan.perform()
