import rclpy

from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import PoseStamped
from pycram_suturo_demo.setup_real_robot import world_setup_with_test_objects
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot
from pycram.robot_plans import (
    ParkArmsActionDescription,
    LookAtActionDescription,
)

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
