import rclpy.exceptions
from pycram.datastructures.enums import Arms
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot
from pycram.robot_plans import ParkArmsAction, ParkArmsActionDescription

from pycram_suturo_demos.helper_methods_and_useful_classes.pickup_helper_methods import (
    initialization,
)
from pycram_suturo_demos.pycram_basic_hsr_demos.A_start_up import setup_hsrb_context

SIMULATED = False
rclpy_node, world, robot_view, context = setup_hsrb_context()
print("hallo marc ich funktioniere")

with real_robot:
    for i in range(10):
        print(str(i) + "execution")
        SequentialPlan(context, ParkArmsActionDescription(Arms.LEFT)).perform()
