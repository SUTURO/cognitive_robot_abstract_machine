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

from pycram.tf_transformations import quaternion_from_euler, quaternion_multiply
import numpy as np

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


def change_orientation(start_pose: PoseStamped) -> PoseStamped:
    """
    Rotate a pose by 180 degrees around the z-axis using quaternion multiplication.

    This is a pure mathematical function that does not require Nav2 initialization.
    It works in unknown environments without a loaded world model.

    :param start_pose: The pose to rotate around (geometry_msgs.msg.PoseStamped).
    :return: A new pose with the same position but rotated 180 degrees.
    """
    quat_orientation = (
        start_pose.pose.orientation.x,
        start_pose.pose.orientation.y,
        start_pose.pose.orientation.z,
        start_pose.pose.orientation.w,
    )

    # 180-degree rotation around z-axis
    quat_add = quaternion_from_euler(0.0, 0.0, np.pi)
    q_new = quaternion_multiply(quat_orientation, quat_add)

    new_pose = PoseStamped()
    new_pose.header = start_pose.header
    new_pose.pose.position = start_pose.pose.position
    new_pose.pose.orientation.x = q_new[0]
    new_pose.pose.orientation.y = q_new[1]
    new_pose.pose.orientation.z = q_new[2]
    new_pose.pose.orientation.w = q_new[3]

    logger.info(
        f"Rotated pose 180 degrees: original quat={quat_orientation}, new quat={tuple(q_new)}"
    )
    return new_pose


def min_distance_2_human(
    human_pose: PoseStamped, robot_pose: PoseStamped, min_distance: float
) -> PoseStamped:
    """
    Calculate a target pose for the robot that maintains a minimum distance from a human.

    The function calculates a position that is at the specified minimum distance from the human,
    along the line connecting the robot's current position to the human's position.
    The robot will be oriented towards the human.

    :param human_pose: The pose of the human (geometry_msgs.msg.PoseStamped).
    :param robot_pose: The current pose of the robot (geometry_msgs.msg.PoseStamped).
    :param min_distance: The minimum distance (in meters) to maintain from the human.
    :return: A new target pose that maintains the specified distance from the human.
    """
    # Extract positions
    human_x = human_pose.pose.position.x
    human_y = human_pose.pose.position.y
    human_z = human_pose.pose.position.z

    robot_x = robot_pose.pose.position.x
    robot_y = robot_pose.pose.position.y
    robot_z = robot_pose.pose.position.z

    # Calculate direction vector from human to robot
    dx = robot_x - human_x
    dy = robot_y - human_y
    dz = robot_z - human_z

    # Calculate current distance
    current_distance = np.sqrt(dx**2 + dy**2)

    if current_distance < 0.001:
        # If robot is at the same position as human, move it back by min_distance in negative x direction
        logger.warning(
            "Robot and human positions are too close, moving robot back along x-axis"
        )
        dx = -min_distance
        dy = 0.0
        current_distance = min_distance

    # Normalize the direction vector (in xy-plane)
    direction_x = dx / current_distance
    direction_y = dy / current_distance

    # Calculate target position at min_distance from human
    target_x = human_x + direction_x * min_distance
    target_y = human_y + direction_y * min_distance
    target_z = robot_z  # Keep the same z-level as robot

    # Calculate orientation to face the human
    # The robot should look towards the human (opposite of the direction vector)
    angle_to_human = np.arctan2(-direction_y, -direction_x)
    target_quat = quaternion_from_euler(0.0, 0.0, angle_to_human)

    # Create target pose
    target_pose = PoseStamped()
    target_pose.header = robot_pose.header
    target_pose.pose.position.x = target_x
    target_pose.pose.position.y = target_y
    target_pose.pose.position.z = target_z
    target_pose.pose.orientation.x = target_quat[0]
    target_pose.pose.orientation.y = target_quat[1]
    target_pose.pose.orientation.z = target_quat[2]
    target_pose.pose.orientation.w = target_quat[3]

    logger.info(
        f"Calculated target pose with {min_distance}m distance from human. "
        f"Human at ({human_x:.2f}, {human_y:.2f}), target at ({target_x:.2f}, {target_y:.2f})"
    )

    return target_pose
