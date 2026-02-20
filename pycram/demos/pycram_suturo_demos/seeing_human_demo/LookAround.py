from time import sleep
import time

import rclpy

from suturo_resources.suturo_map import load_environment
from pycram.external_interfaces import robokudo
from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import PoseStamped
from demos.pycram_suturo_demos.helper_methods_and_useful_classes.robot_setup import (
    robot_setup,
)
from pycram.ros_utils.text_to_image import TextToImagePublisher
from semantic_digital_twin.datastructures.definitions import TorsoState
from pycram.language import SequentialPlan
from pycram.motion_executor import simulated_robot, real_robot
from pycram.robot_plans import (
    ParkArmsActionDescription,
    LookAtActionDescription,
)

rclpy.init()
result = robot_setup()
world, robot_view, context = (
    result.world,
    result.robot_view,
    result.context,
)

camera_frame = robot_view.get_default_camera().root
base_frame = world.get_body_by_name("base_link")

plan = SequentialPlan(
    context,
    ParkArmsActionDescription(Arms.BOTH),
    LookAtActionDescription(
        [PoseStamped.from_list([2.0081, 2.2012, 0.74911], frame=world.root)]
    ),
)

look_left = SequentialPlan(
    context,
    LookAtActionDescription([PoseStamped.from_list([0.1, 1, 0.75], frame=base_frame)]),
)

look_right = SequentialPlan(
    context,
    LookAtActionDescription([PoseStamped.from_list([0.1, 1, 0.75], frame=base_frame)]),
    LookAtActionDescription([PoseStamped.from_list([0.1, -1, 0.75], frame=base_frame)]),
)
look_back = SequentialPlan(
    context,
    LookAtActionDescription([PoseStamped.from_list([-1, 0, 0.75], frame=base_frame)]),
)

look_front = SequentialPlan(
    context,
    ParkArmsActionDescription(Arms.BOTH),
    LookAtActionDescription([PoseStamped.from_list([1.0, 0, 0.75], frame=base_frame)]),
)

with simulated_robot:
    look_left.perform()
    look_right.perform()
    look_back.perform()
    look_front.perform()
