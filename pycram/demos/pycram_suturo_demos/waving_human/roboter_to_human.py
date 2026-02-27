import os

from pycram.datastructures.pose import PoseStamped
from pycram.external_interfaces import nav2_move
import logging

from demos.pycram_suturo_demos.pycram_basic_hsr_demos.start_up import setup_hsrb_context
from pycram.external_interfaces.nav2_move import buffer_to_pose
from waving_continuous import ContinuousWavingDetector

logging.getLogger(__name__).setLevel(logging.WARN)

logger = logging.getLogger(__name__)
rclpy_node, world, robot_view, context = setup_hsrb_context()


detector = ContinuousWavingDetector(retry_interval=1.0)
HUMAN_POSE = detector.wait_for_waving_human()


def robot_move(target_pose: PoseStamped, frame_id: str = "map"):
    """
    Sends a navigation goal to Nav2.
    """
    os.environ["ROS_PYTHON_CHECK_FIELDS"] = "1"
    goal = buffer_to_pose(
        target_pose,
        0.5,
    )
    goal.header.frame_id = frame_id
    print(f"Moving to {goal}")
    nav2_move.start_nav_to_pose(goal)


robot_move(HUMAN_POSE)
