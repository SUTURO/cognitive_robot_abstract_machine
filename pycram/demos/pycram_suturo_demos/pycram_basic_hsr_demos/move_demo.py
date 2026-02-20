import os

import rclpy

from pycram.datastructures.pose import PoseStamped
from pycram.external_interfaces import nav2_move
import logging

from pycram.datastructures.enums import Arms
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot
from pycram.robot_plans import ParkArmsActionDescription
from pycram_suturo_demos.pycram_basic_hsr_demos.start_up import setup_hsrb_context
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix

logger = logging.getLogger(__name__)
rclpy_node, world, robot_view, context = setup_hsrb_context()


CABINET = PoseStamped.from_list(
    position=[3.8683114051818848, 5.459158897399902, 0.0],
    orientation=[0.0, 0.0, 0.04904329912700753, 0.9987966533838301],
    frame=world.root,
)
POPCORN_TABLE = PoseStamped.from_list(
    position=[1.3, 5.3, 0.0],
    orientation=[0.0, 0.0, 0.72, 0.64],
    frame=world.root,
)


def robot_move(target_pose: PoseStamped, frame_id: str = "map"):
    """
    Sends a navigation goal to Nav2.
    """
    os.environ["ROS_PYTHON_CHECK_FIELDS"] = "1"
    goal = target_pose.ros_message()
    print(f"Moving to {goal}'")

    nav2_move.start_nav_to_pose(goal)


robot_move(POPCORN_TABLE)
