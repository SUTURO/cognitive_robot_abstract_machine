import time

import rclpy

from demos.pycram_suturo_demos.helper_methods_and_useful_classes.A_robot_setup import (
    robot_setup,
)
from pycram.datastructures.enums import Arms
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot, simulated_robot
from pycram.robot_plans import ParkArmsActionDescription

rclpy.init()
simulated: bool = True
robot_type = simulated_robot if simulated else real_robot
with_objects: bool = True

result = robot_setup(simulation=simulated, with_simulated_objects=with_objects)
world, context, robot_view = (result.world, result.context, result.robot_view)

plan = SequentialPlan(context, ParkArmsActionDescription(Arms.BOTH))

with robot_type:
    plan.perform()
    time.sleep(100)
