import os

from pycram.datastructures.pose import PoseStamped
from pycram.external_interfaces import nav2_move
import logging

from pycram_suturo_demos.pycram_basic_hsr_demos.start_up import setup_hsrb_context
from waving_continuous import ContinuousWavingDetector

logger = logging.getLogger(__name__)
rclpy_node, world, robot_view, context = setup_hsrb_context()


def robot_move(target_pose: PoseStamped, frame_id: str = "map"):
    """
    Sends a navigation goal to Nav2.
    """
    os.environ["ROS_PYTHON_CHECK_FIELDS"] = "1"
    goal = target_pose.ros_message()
    goal.header.frame_id = frame_id
    print(f"Moving to {goal}")
    nav2_move.start_nav_to_pose(goal)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    detector = ContinuousWavingDetector(retry_interval=1.0)
    human_pose = detector.wait_for_waving_human()

    if human_pose is not None:
        robot_move(human_pose, frame_id="map")
    else:
        logger.error("No waving human found – not moving")
