import os

import semantic_digital_twin
from pycram.datastructures.pose import PoseStamped
from pycram.external_interfaces import nav2_move
import logging

from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot, simulated_robot
from pycram.robot_plans import NavigateActionDescription
from demos.pycram_suturo_demos.pycram_basic_hsr_demos.start_up import setup_hsrb_context

logging.getLogger(semantic_digital_twin.world.__name__).setLevel(logging.WARN)

logger = logging.getLogger(__name__)
rclpy_node, world, robot_view, context = setup_hsrb_context()

USE_REAL_ROBOT = False

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
    Sends a navigation goal to Nav2 (real robot) or executes it in simulation.
    """
    buffered_pose = nav2_move.buffer_to_pose(target_pose, 0.5)

    if USE_REAL_ROBOT:
        os.environ["ROS_PYTHON_CHECK_FIELDS"] = "1"
        goal = buffered_pose.ros_message()
        logger.info(f"[REAL] Moving to {goal}")
        with real_robot:
            nav2_move.start_nav_to_pose(goal)
    else:
        logger.info(f"[SIM] Moving to {buffered_pose}")
        plan = SequentialPlan(
            context,
            NavigateActionDescription(target_location=buffered_pose),
        )
        with simulated_robot:
            plan.perform()


robot_move(CABINET)
