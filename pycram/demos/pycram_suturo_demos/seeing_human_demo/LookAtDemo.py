from time import sleep
import time

import rclpy

from suturo_resources.suturo_map import load_environment
from pycram.external_interfaces import robokudo
from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import PoseStamped
from demos.pycram_suturo_demos.helper_methods_and_useful_classes.real_setup import (
    world_setup_with_test_objects,
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
result = world_setup_with_test_objects()
world, robot_view, context = (
    result.world,
    result.robot_view,
    result.context,
)
x: int | None = None
y: int | None = None
z: int | None = None

plan = SequentialPlan(
    context,
    ParkArmsActionDescription(Arms.BOTH),
    LookAtActionDescription([PoseStamped.from_list([x, y, z], frame=world.root)]),
)

with real_robot:
    text_pub = TextToImagePublisher()
    found_position = True
    timeout = time.time() + 60
    internaltimeout = 15
    # While human is seen print location
    while found_position and time.time() < timeout:
        # Send goal
        position = robokudo.query_current_human_position_in_continues()
        if (
            position is not None
            and position.header.stamp.sec > time.time() - internaltimeout
        ):
            global x, y, z
            x = round(position.point.x, 2)
            y = round(position.point.y, 2)
            z = round(position.point.z, 2)
            text_pub.publish_text(f"Point x: {x} y: {y} z: {z}")
            plan.perform()
        else:
            text_pub.publish_text("No Human seen.")
            found_position = False
        sleep(0.5)

    # Close every think
    text_pub.publish_text("Shutting down ...")
    robokudo.shutdown_robokudo_interface()
    rclpy.shutdown()
