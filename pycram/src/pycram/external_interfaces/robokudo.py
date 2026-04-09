import sys
import logging
import threading
from threading import Thread

from rclpy.action.client import ClientGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.action import ActionClient
from pycram.ros import create_action_client
from robokudo_msgs.action import Query
from typing_extensions import List, Callable, Optional
from typing import Any
from ..datastructures.pose import PoseStamped
from ..designator import ObjectDesignatorDescription
from geometry_msgs.msg import PointStamped
from time import sleep
import time

logger = logging.getLogger(__name__)

# Global variables for shared resources
robokudo_action_client: ActionClient | None = None
robokudo_node: Node | None = None
chd: Node | None = None
current_human_position: PoseStamped | None = None
is_init = False
goal_handle: ClientGoalHandle | None = None
executor: MultiThreadedExecutor | None = None
executor_thread: Thread | None = None
continues_lock = False


def create_robokudo_action_client():
    """Creates a new ActionClient and a MultithreadedExecutor for the Robokudo interface."""

    global robokudo_node, executor, executor_thread, chd

    if robokudo_node is None:
        robokudo_node = Node("robokudo_interface_client")

    if chd is None:
        chd = CONTINUOUS_HUMAN_DETECTION()

    client = create_action_client("/robokudo/query", Query, robokudo_node)

    if executor is None:
        executor = MultiThreadedExecutor()
        executor.add_node(robokudo_node)
        executor.add_node(chd)

        executor_thread = Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        logger.info("Started MultiThreadedExecutor")

    if not client.wait_for_server(timeout_sec=10.0):
        logger.error("robokudo_query action server not available")
        return None

    logger.info("Action server is available")
    return client


def init_robokudo_interface(func: Callable) -> Callable:
    """Initializes the Robokudo interface"""

    def wrapper(*args, **kwargs):
        global is_init, robokudo_action_client

        if (
            is_init
            and robokudo_action_client is not None
            and robokudo_action_client.wait_for_server(timeout_sec=10.0)
        ):
            return func(*args, **kwargs)
        elif (
            is_init
            and robokudo_action_client is not None
            and robokudo_action_client.wait_for_server(timeout_sec=10.0) == False
        ):
            logger.warning(
                "Robokudo node is not available anymore, could not initialize robokudo interface"
            )
            is_init = False
            return None

        if "robokudo_msgs" not in sys.modules:
            logger.warning(
                "Could not initialize the Robokudo interface since the robokudo_msgs are not imported"
            )
            return None

        robokudo_action_client = create_robokudo_action_client()
        if robokudo_action_client is None:
            logger.warning("Could not create robokudo action client")
            is_init = False
            return None
        logger.info("Successfully initialized Robokudo interface")
        is_init = True

        return func(*args, **kwargs)

    return wrapper


@init_robokudo_interface
def send_query(
    obj_type: Optional[str] = None,
    region: Optional[str] = None,
    attributes: Optional[List[str]] = None,
) -> Any:
    """Generic function to send a query to RoboKudo."""
    if continues_lock:
        logger.warning("Can not send query while continuous query is running")
        return None
    else:
        global robokudo_action_client, goal_handle
        goal = Query.Goal()
        continues = False

        if obj_type:
            goal.obj.type = obj_type
        if region:
            goal.obj.location = region
        if attributes:
            goal.obj.attribute = attributes
            for a in attributes:
                if a == "continues":
                    continues = True

        result: Query.Result | None = None
        result_event = threading.Event()

        def goal_response_callback(future):
            global goal_handle
            goal_handle = future.result()
            if not goal_handle.accepted:
                logger.info("Goal rejected")
                return
            logger.info("Goal accepted")
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(get_result_callback)

        def get_result_callback(future):
            nonlocal result
            result = future.result().result
            logger.info("Finished ")
            result_event.set()

        logger.info("Send Query")
        if continues:
            send_goal_future = robokudo_action_client.send_goal_async(goal)
            send_goal_future.add_done_callback(goal_response_callback)
        else:
            send_goal_future = robokudo_action_client.send_goal_async(goal)
            send_goal_future.add_done_callback(goal_response_callback)
            result_event.wait()
            goal_handle = None

        return result


@init_robokudo_interface
def query_human() -> "PointStamped":
    """Query RoboKudo for human detection and return the detected human's pose.
    This only querys one human.
    """
    result = send_query(obj_type="human")
    posi = None
    if result is None:
        return None
    else:
        for r in result.res:
            for p in r.pose:
                if posi is None or p.pose > posi:
                    posi = p.pose
    return posi


@init_robokudo_interface
def query_current_human_position_in_continues():
    global current_human_position, continues_lock
    if not goal_handle:
        # Send goal
        send_query(obj_type="human", attributes=["continues"])
        continues_lock = True
        timeout = time.time() + 5
        while current_human_position is None and time.time() < timeout:
            sleep(0.1)
    return current_human_position


@init_robokudo_interface
def query_all_objects() -> dict:
    return send_query(obj_type="object")


@init_robokudo_interface
def query_postion_closest_object() -> PoseStamped:
    result = send_query()
    posi = None
    if result is None:
        return None
    else:
        for r in result.res:
            print(r.type)
            for p in r.pose:
                if posi is None or p.pose > posi:
                    posi = p.pose
    return posi


@init_robokudo_interface
def query_object(obj_desc: ObjectDesignatorDescription) -> dict:
    return send_query(obj_type=str(obj_desc.types[0]))


@init_robokudo_interface
def query_object_str(obj_desc: str) -> dict:
    return send_query(obj_type=str(obj_desc))


"""
Cancel needs to be implemented bevor this will work.
@init_robokudo_interface
def stop_query():
    global robokudo_action_client
    robokudo_action_client.cancel_goal_async()
    logger.info("Cancelled current Robokudo query goal")

"""


@init_robokudo_interface
def query_specific_region(region: str) -> Any:
    return send_query(region=region)


@init_robokudo_interface
def query_human_attributes() -> Any:
    return send_query(obj_type="human", attributes=["attributes"])


@init_robokudo_interface
def query_waving_human() -> Optional[PoseStamped]:
    result = send_query(obj_type="human", attributes=["waving"])
    if result is None:
        logger.warning("query_waving_human: no result from RoboKudo")
        return None
    try:
        ros_pose_stamped = result.res[0].pose[0]
    except (AttributeError, IndexError, TypeError):
        logger.warning("query_waving_human: no pose in result: %s", result)
        return None
    return PoseStamped.from_ros_message(ros_pose_stamped)


@init_robokudo_interface
def cancel_goal():
    """Sends a cancel request for the active goal."""
    if goal_handle is None:
        logger.error("No active goal to cancel.")
        return

    logger.info("Sending cancel request...")
    cancel_future = goal_handle.cancel_goal_async()
    cancel_future.add_done_callback(cancel_done_callback)


def cancel_done_callback(future):
    """Handles the response from the action server regarding goal cancellation."""
    global goal_handle, current_human_position, continues_lock
    cancel_response = future.result()
    if len(cancel_response.goals_canceling) > 0:
        logger.info("Goal cancellation accepted by the server.")
        goal_handle = None
        current_human_position = None
        continues_lock = False
    else:
        logger.warning("Goal cancellation was not successful.")
    logger.info("Shutting down after cancellation is accepted.")


def shutdown_robokudo_interface():
    """Clean shutdown of perception interface."""

    global robokudo_node, executor, executor_thread, is_init, chd, current_human_position

    cancel_goal()

    if executor is not None:
        executor.shutdown()

    if executor_thread is not None:
        executor_thread.join(timeout=5)

    if robokudo_node is not None:
        robokudo_node.destroy_node()

    if chd is not None:
        chd.destroy_node()

    is_init = False
    logger.info("Robokudo interface shut down")


class CONTINUOUS_HUMAN_DETECTION(Node):
    def __init__(self):

        super().__init__("continous_human_detection")

        self.sub_chd = self.create_subscription(
            PointStamped, "/human_pose", self.data_callback, 10
        )

    def data_callback(self, data):
        global current_human_position
        if data is not None:
            current_human_position = data
