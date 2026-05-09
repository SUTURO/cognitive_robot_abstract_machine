import logging
import threading
from threading import Thread
from typing import Callable
from action_msgs.srv import CancelGoal_Response
from rclpy import Future
from rclpy.action.client import ClientGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose, NavigateToPose_GetResult_Response
from geometry_msgs.msg import PoseStamped

from pycram.ros import create_action_client

logger = logging.getLogger(__name__)

# Global variables for shared resources
nav_action_client: ActionClient | None = None
nav_node: Node | None = None
is_init = False
current_goal_handle: ClientGoalHandle | None = None
executor: MultiThreadedExecutor | None = None
executor_thread: Thread | None = None


def create_nav_action_client():
    """Creates a new ActionClient and a MultithreadedExecutor for the NavigateToPose interface."""

    global nav_node, executor, executor_thread

    if nav_node is None:
        nav_node = Node("nav_interface_client")

    client = create_action_client("navigate_to_pose", NavigateToPose, nav_node)

    if executor is None:
        executor = MultiThreadedExecutor()
        executor.add_node(nav_node)

        executor_thread = Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        logger.info("Started MultiThreadedExecutor")

    if not client.wait_for_server(timeout_sec=10.0):
        logger.error("navigate_to_pose action server not available")
        return None

    return client


def init_nav_interface(func: Callable) -> Callable:
    """Ensures initialization of the navigation interface before function execution."""

    def wrapper(*args, **kwargs):
        global is_init, nav_action_client

        if is_init:
            return func(*args, **kwargs)

        try:
            nav_action_client = create_nav_action_client()

            if nav_action_client is None:
                logger.warning("Could not create navigation action client")
                return None

            logger.info("Successfully initialized navigation interface")
            is_init = True

        except Exception as e:
            logger.error(f"Failed to initialize navigation interface: {e}")
            return None

        return func(*args, **kwargs)

    return wrapper


@init_nav_interface
def start_nav_to_pose(nav_pose: PoseStamped) -> NavigateToPose.Result | None:
    """Sends a goal to the NavigateToPose action, initiating robot navigation to a given pose.

    :param nav_pose: The goal position
    :type nav_pose: PoseStamped
    :returns: a NavigateToPose_Result or None if there was an error getting the result
    :rtype: NavigateToPose.Result | None
    """

    global nav_action_client, current_goal_handle, nav_node

    result_response: NavigateToPose_GetResult_Response | None = None
    result: NavigateToPose.Result | None = None
    result_event = threading.Event()

    def goal_response_callback(future):
        goal_handle: ClientGoalHandle
        goal_handle = future.result()
        if not goal_handle.accepted:
            logger.error("Goal rejected by navigation server")
            result_event.set()
            return

        logger.info("Sent query to navigate_to_pose")
        global current_goal_handle
        current_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(result_callback)

    def result_callback(future):
        nonlocal result_response, result
        result_response = future.result()
        result = result_response.result
        logger.info(
            f"Finished navigation with response status: {result_response.status} and result code: {result.error_code}"
        )
        result_event.set()

    goal_msg = NavigateToPose.Goal()
    goal_msg.pose = nav_pose

    send_future = nav_action_client.send_goal_async(goal_msg)
    send_future.add_done_callback(goal_response_callback)

    result_event.wait()

    return result


@init_nav_interface
def cancel_current_goal() -> CancelGoal_Response | None:
    """Cancels the current navigation goal.

    :returns: a CancelGoal_Response or None if there is no current goal
    :rtype: CancelGoal_Response | None
    """

    global current_goal_handle, nav_node

    if current_goal_handle is None:
        logger.warning("No active goal to cancel")
        return None

    logger.info("Canceling navigation goal")
    cancel_event = threading.Event()
    cancel_response: CancelGoal_Response | None = None

    def cancel_callback(future: Future):
        nonlocal cancel_response
        cancel_response = future.result()
        cancel_event.set()

    cancel_future: Future
    cancel_future = current_goal_handle.cancel_goal_async()
    cancel_future.add_done_callback(cancel_callback)

    cancel_event.wait()

    if cancel_response.return_code == CancelGoal_Response.ERROR_NONE:
        logger.info("Canceled current goal")
        return cancel_response

    logger.warning("Failed to cancel current goal")
    return_code = cancel_response.return_code
    if return_code == CancelGoal_Response.ERROR_REJECTED:
        logger.warning("Goal cancellation was rejected")
    elif return_code == CancelGoal_Response.ERROR_GOAL_TERMINATED:
        logger.warning("Goal already terminated")
    elif return_code == CancelGoal_Response.ERROR_UNKNOWN_GOAL_ID:
        logger.warning("Unknown goal ID")
    return cancel_response


def shutdown_nav_interface():
    """Clean shutdown of navigation interface."""

    global nav_node, executor, executor_thread, is_init

    if executor is not None:
        executor.shutdown()

    if executor_thread is not None:
        executor_thread.join(timeout=5)

    if nav_node is not None:
        nav_node.destroy_node()

    is_init = False
    logger.info("Navigation interface shut down")
