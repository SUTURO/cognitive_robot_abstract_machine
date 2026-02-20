import rclpy

from demos.helper_methods_and_useful_classes.robot_setup import robot_setup
from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot, simulated_robot
from pycram.robot_plans import (
    ParkArmsActionDescription,
    LookAtActionDescription,
)
from demos.helper_methods_and_useful_classes.robot_setup import robot_setup

rclpy.init()
result = robot_setup()
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

with simulated_robot:
    plan.perform()
