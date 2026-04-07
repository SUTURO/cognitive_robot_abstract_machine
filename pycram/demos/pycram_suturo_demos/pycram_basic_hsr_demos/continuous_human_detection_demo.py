import rclpy
from pycram.robot_plans import LookAtAction

from pycram.language import SequentialPlan

from pycram.external_interfaces import robokudo
import time
from time import sleep
from pycram.ros_utils.text_to_image import TextToImagePublisher
import logging

from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot
from pycram.robot_plans import ParkArmsActionDescription
from pycram_suturo_demos.pycram_basic_hsr_demos.A_start_up import setup_hsrb_context
from semantic_digital_twin.adapters.ros import (
    HomogeneousTransformationMatrixToRos2Converter,
)

logger = logging.getLogger(__name__)

rclpy_node, world, robot_view, context = setup_hsrb_context()


def main():
    """Testing"""
    text_pub = TextToImagePublisher()
    found_position = True
    timeout = time.time() + 20
    internaltimeout = 15
    # While human is seen print location
    while found_position and time.time() < timeout:
        # Send goal
        position = robokudo.query_current_human_position_in_continues()
        if (
            position is not None
            and position.header.stamp.sec > time.time() - internaltimeout
        ):
            x = round(position.point.x, 2)
            y = round(position.point.y, 2)
            z = round(position.point.z, 2)
            text_pub.publish_text(f"Point x: {x} y: {y} z: {z}")
        else:
            text_pub.publish_text("No Human seen.")
            found_position = False
        sleep(0.5)

    # Close every think
    robokudo.shutdown_robokudo_interface()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
