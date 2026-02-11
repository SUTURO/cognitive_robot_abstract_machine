import threading
import time
import logging
from enum import Enum
from typing import Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from suturo_resources.suturo_map import load_environment

from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.dataclasses import Context
from pycram.external_interfaces import nav2_move
from pycram.ros_utils.text_to_image import TextToImagePublisher
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
    VizMarkerPublisher,
)
from semantic_digital_twin.robots.hsrb import HSRB
import pycram.alternative_motion_mappings.hsrb_motion_mapping  # noqa: F401 (side-effect registration)

# New: perception
from pycram.external_interfaces import robokudo

logger = logging.getLogger(__name__)


class DriveToPosition(Enum):
    CABINET = PoseStamped.from_list(
        position=[3.8683114051818848, 5.459158897399902, 0.0],
        orientation=[0.0, 0.0, 0.04904329912700753, 0.9987966533838301],
    )


def _sleep_ros(node, seconds: float) -> None:
    """ROS2-friendly sleep that keeps callbacks/executors responsive."""
    rclpy.spin_once(node, timeout_sec=float(seconds))


def _iter_head_pan_positions():
    """Simple left-to-right scan values for head_pan_joint (HSRB)."""
    # keep it local+simple: same range as restaurant_demo02
    return [-0.5, 0.0, 0.5, 1.0, 0.5, 0.0]


def search_waving_human_pose(
    node,
    *,
    scan_head: bool = True,
    retry_sleep_s: float = 0.5,
) -> Optional[PoseStamped]:
    """Query RoboKudo for a waving human until we get a pose.

    Contract:
      - returns a PoseStamped when found
      - keeps searching indefinitely (as requested)

    Notes:
      - RoboKudo may return poses in an optical/camera frame; for navigation we
        set the frame to 'map' on the outgoing goal (same behavior as before).
    """
    # lazy import to avoid pulling in heavy deps when this file is imported for tests
    try:
        from pycram.robot_plans import MoveJointsMotion
    except Exception:
        MoveJointsMotion = None

    text_pub = TextToImagePublisher(node=node)
    text_pub.publish_text("Searching for a waving human…")

    while rclpy.ok():
        if scan_head and MoveJointsMotion is not None:
            for pan in _iter_head_pan_positions():
                try:
                    MoveJointsMotion(["head_pan_joint"], [pan]).perform()
                except Exception as e:
                    # head motion is optional; don't fail perception for it
                    logger.debug("Head scan motion failed: %s", e)

                try:
                    pose = robokudo.query_waving_human()
                except Exception as e:
                    logger.debug("RoboKudo query failed: %s", e)
                    pose = None

                if pose is not None:
                    text_pub.publish_text("Found waving human")
                    return pose

                _sleep_ros(node, retry_sleep_s)
        else:
            # no scan: just query with a sleep
            try:
                pose = robokudo.query_waving_human()
            except Exception as e:
                logger.debug("RoboKudo query failed: %s", e)
                pose = None

            if pose is not None:
                text_pub.publish_text("Found waving human")
                return pose
            _sleep_ros(node, retry_sleep_s)

    return None


def real_demo():
    rclpy.init()

    node = rclpy.create_node("detect_human_and_drive")
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    thread = threading.Thread(target=executor.spin, daemon=True, name="rclpy-executor")
    thread.start()
    time.sleep(0.1)

    # world from giskard (if available). If not available, keep the demo runnable by falling back.
    hsrb_world = fetch_world_from_service(node)
    if hsrb_world is None:
        logger.warning(
            "No 'fetch_world' Trigger service found in the ROS graph; "
            "falling back to local environment world."
        )
        hsrb_world = load_environment()
    else:
        try:
            hsrb_world.get_body_by_name("environment")
        except Exception as e:
            logger.debug(e)
            env_world = load_environment()
            with hsrb_world.modify_world():
                hsrb_world.merge_world(env_world)

    VizMarkerPublisher(hsrb_world, node, use_visuals=True)

    context = Context(
        hsrb_world, hsrb_world.get_semantic_annotations_by_type(HSRB)[0], ros_node=node
    )

    # 1) find waving human pose (blocks until found)
    human_pose = search_waving_human_pose(node, scan_head=True)

    # 2) navigate to that pose
    text_pub = TextToImagePublisher(node=node)
    if human_pose is None:
        text_pub.publish_text("No waving human pose found (shutdown)")
        rclpy.shutdown()
        return

    text_pub.publish_text("Driving to waving human")
    goal = human_pose.ros_message()
    goal.header.frame_id = "map"
    nav2_move.start_nav_to_pose(goal)

    rclpy.shutdown()


if __name__ == "__main__":
    real_demo()
