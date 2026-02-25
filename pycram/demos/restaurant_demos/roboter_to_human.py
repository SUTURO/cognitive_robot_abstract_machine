import os
import sys

from pycram.datastructures.pose import PoseStamped
from pycram.external_interfaces import nav2_move, robokudo
import logging

try:
    # Used when demos are on the Python path (common when running from the pycram/demos folder).
    from pycram_suturo_demos.pycram_basic_hsr_demos.start_up import setup_hsrb_context
except ImportError:
    # Fallback for IDE/typechecking: add the local demos folder to sys.path.
    _here = os.path.abspath(os.path.dirname(__file__))
    _demos_root = os.path.abspath(os.path.join(_here, ".."))
    # We need the *pycram/demos* folder on sys.path so that `pycram_suturo_demos` is importable.
    if _demos_root not in sys.path:
        sys.path.insert(0, _demos_root)
    from pycram_suturo_demos.pycram_basic_hsr_demos.start_up import setup_hsrb_context

logger = logging.getLogger(__name__)
rclpy_node, world, robot_view, context = setup_hsrb_context()


def query_waving_human_pose(
    target_frame: str = "map",
    tf_timeout: float = 5.0,
) -> PoseStamped | None:
    """Query RoboKudo for a waving human and return its pose.

    RoboKudo result contains a list of objects and each object can contain multiple poses.
    We take the first returned object's first pose.

    :param target_frame: Frame in which the pose should be returned (default: map).
    :param tf_timeout: Timeout (s) for tf transform.
    :return: Human pose as PyCRAM PoseStamped or None if not found.
    """

    result = robokudo.send_query(obj_type="human", attributes=["waving"])
    if result is None:
        logger.warning("Robokudo query returned None")
        return None

    try:
        ros_pose_stamped = result.res[0].pose[0]
    except (AttributeError, IndexError, TypeError):
        logger.warning("No waving human pose in Robokudo result: %s", result)
        return None

    pose = PoseStamped.from_ros_message(ros_pose_stamped)

    if target_frame and pose.frame_id and str(pose.frame_id) != str(target_frame):
        try:
            # Lazy import to keep this demo usable without TF installed in pure unit contexts.
            from semantic_digital_twin.adapters.ros.tfwrapper import TFWrapper

            tfw = TFWrapper(node=rclpy_node)
            transformed = tfw.transform_pose(target_frame, ros_pose_stamped, tf_timeout)
            pose = PoseStamped.from_ros_message(transformed)
        except Exception as e:
            logger.warning(
                "Failed to transform human pose from '%s' to '%s': %s",
                pose.frame_id,
                target_frame,
                e,
            )

    return pose


def robot_move(target_pose: PoseStamped, frame_id: str = "map"):
    """Sends a navigation goal to Nav2."""

    os.environ["ROS_PYTHON_CHECK_FIELDS"] = "1"
    goal = target_pose.ros_message()
    if frame_id:
        goal.header.frame_id = frame_id

    logger.info("Moving to %s", goal)
    nav2_move.start_nav_to_pose(goal)


def main():
    # 1) Query for waving human
    human_pose = query_waving_human_pose(target_frame="map")
    if human_pose is None:
        logger.error("No waving human found - aborting navigation")
        return

    # 2) Navigate to human pose
    robot_move(human_pose, frame_id="map")


if __name__ == "__main__":
    main()
