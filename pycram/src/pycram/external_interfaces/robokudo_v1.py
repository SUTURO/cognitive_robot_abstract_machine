import sys
import logging
from threading import Lock, RLock
from typing import Any, Optional, Callable

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from typing_extensions import List

from robokudo_msgs.action import Query
from robokudo_msgs.msg import ObjectDesignator as RobokudoObjectDesignator
from geometry_msgs.msg import PointStamped

from ..datastructures.pose import PoseStamped
from ..designator import ObjectDesignatorDescription


logger = logging.getLogger(__name__)

robokudo_found = False
is_init = False
client: ActionClient = None

robokudo_lock = Lock()
robokudo_rlock = RLock()

# ROS2 requires a node instance for everything
rclpy.init()
_ros2_node = Node("robokudo_interface")


def thread_safe(func: Callable) -> Callable:
    """
    Adds thread safety to a function via a decorator. This uses the robokudo_lock

    :param func: Function that should be thread safe
    :return: A function with thread safety
    """

    def wrapper(*args, **kwargs):
        with robokudo_rlock:
            return func(*args, **kwargs)

    return wrapper


def init_robokudo_interface(func: Callable) -> Callable:
    """
    Checks if the ROS messages are available and if Robokudo is running,
    if that is the case the interface will be initialized.
    """

    def wrapper(*args, **kwargs):

        global is_init
        global client

        # Check messages
        if "robokudo_msgs" not in sys.modules:
            logger.warning("robokudo_msgs not imported – cannot init")
            return

        # Check node existence
        node_names = _ros2_node.get_node_names()

        if is_init and "robokudo" in node_names:
            return func(*args, **kwargs)

        elif is_init and "robokudo" not in node_names:
            logger.warning(
                "Robokudo node is not available anymore, could not initialize robokudo interface"
            )
            is_init = False
            return

        # Initialize now
        if "robokudo" in node_names:
            logger.info("Initializing Robokudo interface (ROS2)")
            client = ActionClient(_ros2_node, Query, "robokudo/query")

            logger.info("Waiting for Robokudo action server…")
            if not client.wait_for_server(timeout_sec=5.0):
                logger.warning("Robokudo action server NOT available")
                return

            is_init = True
            logger.info("Robokudo interface initialized")
        else:
            logger.warning("Robokudo not running")
            return

        return func(*args, **kwargs)

    return wrapper


# ==============================================================================
# ACTION CALL
# ==============================================================================


@init_robokudo_interface
def send_query(
    obj_type: Optional[str] = None,
    region: Optional[str] = None,
    attributes: Optional[List[str]] = None,
) -> Any:

    goal_msg = Query.Goal()

    # Fill goal
    if obj_type:
        goal_msg.obj.type = obj_type
    if region:
        goal_msg.obj.location = region
    if attributes:
        goal_msg.obj.attribute = attributes

    global client

    # Send goal asynchronously
    future = client.send_goal_async(
        goal_msg, feedback_callback=lambda fb: logger.info(f"Feedback: {fb}")
    )

    rclpy.spin_until_future_complete(_ros2_node, future)
    goal_handle = future.result()

    if not goal_handle.accepted:
        logger.warning("Robokudo goal rejected")
        return None

    logger.info("Robokudo goal accepted, waiting for result…")

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(_ros2_node, result_future)

    return result_future.result().result  # Query.Result


# ==============================================================================
# API
# ==============================================================================


@init_robokudo_interface
def query_all_objects() -> dict:
    return send_query()


@init_robokudo_interface
def query_object(obj_desc: ObjectDesignatorDescription) -> dict:
    return send_query(obj_type=str(obj_desc.types[0]))


@init_robokudo_interface
def query_human() -> PointStamped:
    result = send_query(obj_type="human")
    return result if result else None


@init_robokudo_interface
def stop_query():
    global client
    client.cancel_goal_async()
    logger.info("Cancelled current Robokudo query goal")


@init_robokudo_interface
def query_specific_region(region: str) -> Any:
    return send_query(region=region)


@init_robokudo_interface
def query_human_attributes() -> Any:
    return send_query(obj_type="human", attributes=["attributes"])


@init_robokudo_interface
def query_waving_human() -> PoseStamped:
    result = send_query(obj_type="human")
    if result and result.res:
        try:
            pose = PoseStamped.from_pose_stamped(result.res[0].pose[0])
            return pose
        except IndexError:
            pass
    return None
