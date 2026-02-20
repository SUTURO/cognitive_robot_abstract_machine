import threading
import time
import logging
from enum import Enum

import rclpy
from rclpy.executors import SingleThreadedExecutor
from suturo_resources.suturo_map import load_environment

from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.dataclasses import Context
from pycram.external_interfaces import nav2_move
from pycram.ros_utils.text_to_image import TextToImagePublisher
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.viz_marker import VizMarkerPublisher
from semantic_digital_twin.robots.hsrb import HSRB
import pycram.alternative_motion_mappings.hsrb_motion_mapping

logger = logging.getLogger(__name__)


class DriveToPosition(Enum):
    CABINET = PoseStamped.from_list(
        position=[3.8683114051818848, 5.459158897399902, 0.0],
        orientation=[0.0, 0.0, 0.04904329912700753, 0.9987966533838301],
    )


def real_demo():
    rclpy.init()

    node = rclpy.create_node("drive_to_cabinet")
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    thread = threading.Thread(target=executor.spin, daemon=True, name="rclpy-executor")
    thread.start()
    time.sleep(0.1)

    # world from giskard
    hsrb_world = fetch_world_from_service(node)

    try:
        hsrb_world.get_body_by_name("environment")
    except Exception as e:
        logger.debug(e)
        env_world = load_environment()
        with hsrb_world.modify_world():
            hsrb_world.merge_world(env_world)

    VizMarkerPublisher(hsrb_world, node, throttle_state_updates=5)

    context = Context(
        hsrb_world, hsrb_world.get_semantic_annotations_by_type(HSRB)[0], ros_node=node
    )

    text_pub = TextToImagePublisher()
    for pos in DriveToPosition:
        text_pub.publish_text(f"Moving to: {pos.name}")
        goal = pos.value.ros_message()
        goal.header.frame_id = "map"
        nav2_move.start_nav_to_pose(goal)

    rclpy.shutdown()


real_demo()
