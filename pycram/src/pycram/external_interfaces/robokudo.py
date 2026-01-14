import logging
import threading
from threading import Thread
from rclpy.action.client import ClientGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.action import ActionClient
from pycram.ros import create_action_client
from robokudo_msgs.action import Query
from robokudo_msgs.msg import ObjectDesignator as RobokudoObjectDesignator
from typing_extensions import List, Callable, Optional
from typing import Any
from ..datastructures.pose import PoseStamped
from ..designator import ObjectDesignatorDescription

logger = logging.getLogger(__name__)

# Global variables for shared resources
robokudo_action_client: ActionClient | None = None
robokudo_node: Node | None = None
is_init = False
current_goal_handle: ClientGoalHandle | None = None
executor: MultiThreadedExecutor | None = None
executor_thread: Thread | None = None


def create_robokudo_action_client():
    """Creates a new ActionClient and a MultithreadedExecutor for the Robokudo interface."""

    global robokudo_node, executor, executor_thread

    if robokudo_node is None:
        robokudo_node = Node("robokudo_interface_client")

    client = create_action_client("/robokudo/query", Query, robokudo_node)

    if executor is None:
        executor = MultiThreadedExecutor()
        executor.add_node(robokudo_node)

        executor_thread = Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        logger.info("Started MultiThreadedExecutor")

    if not client.wait_for_server(timeout_sec=10.0):
        logger.error("robokudo_query action server not available")
        return None

    return client


def init_robokudo_interface(func: Callable) -> Callable:
    """Initializes the Robokudo interface"""

    def wrapper(*args, **kwargs):
        global is_init, robokudo_action_client

        if is_init:
            return func(*args, **kwargs)
        try:
            robokudo_action_client = create_robokudo_action_client()

            if robokudo_action_client is None:
                logger.warning("Could not create robokudo action client")
                return None

            logger.info("Successfully initialized robokudo interface")
            is_init = True

        except Exception as e:
            logger.error(f"Failed to initialize robokudo interface: {e}")
            return None

        return func(*args, **kwargs)

    return wrapper


@init_robokudo_interface
def send_query(
    obj_type: Optional[str] = None,
    region: Optional[str] = None,
    attributes: Optional[List[str]] = None,
) -> Any:
    """Generic function to send a query to RoboKudo."""

    global robokudo_action_client
    goal = Query.Goal()

    if obj_type:
        goal.obj.type = obj_type
    if region:
        goal.obj.location = region
    if attributes:
        goal.obj.attribute = attributes

    result: Query.Result | None = None
    result_event = threading.Event()

    def goal_response_callback(future):
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
    send_goal_future = robokudo_action_client.send_goal_async(goal)
    send_goal_future.add_done_callback(goal_response_callback)

    result_event.wait()

    return result


@init_robokudo_interface
def query_human() -> PoseStamped:
    """Query RoboKudo for human detection and return the detected human's pose."""
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
def query_all_objects() -> dict:
    return send_query()


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
def query_waving_human() -> PoseStamped:
    result = send_query(obj_type="human")
    if result and result.res:
        try:
            pose = PoseStamped.from_pose_stamped(result.res[0].pose[0])
            return pose
        except IndexError:
            pass
    return None


@init_robokudo_interface
def cancel_goal():
    """Sends a cancel request for the active goal."""
    if not current_goal_handle:
        logger.error("No active goal to cancel.")
        return

    logger.info("Sending cancel request...")
    cancel_future = current_goal_handle.cancel_goal_async()
    cancel_future.add_done_callback(cancel_done_callback)


def cancel_done_callback(future):
    """Handles the response from the action server regarding goal cancellation."""
    cancel_response = future.result()
    if len(cancel_response.goals_canceling) > 0:
        logger.info("Goal cancellation accepted by the server.")
    else:
        logger.warning("Goal cancellation was not successful.")
    # self.done = True
    logger.info("Shutting down after cancellation is accepted.")


def shutdown_robokudo_interface():
    """Clean shutdown of perception interface."""

    global robokudo_node, executor, executor_thread, is_init

    if executor is not None:
        executor.shutdown()

    if executor_thread is not None:
        executor_thread.join(timeout=5)

    if robokudo_node is not None:
        robokudo_node.destroy_node()

    is_init = False
    logger.info("Navigation interface shut down")
