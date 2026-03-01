"""
waving_and_driving_sim.py – Detect a waving human (or use dummy data)
and drive the HSR-B toward them, keeping a minimum stand-off distance.

Same logic as roboter_to_human.py but falls back to a fixed dummy pose
when RoboKudo is not available.

Set ``SIMULATED`` to switch between simulation and real robot.
"""

import logging
import time
from typing import Optional

import numpy as np
import rclpy

from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import PoseStamped
from pycram.external_interfaces.nav2_move import min_distance_2_human
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot, simulated_robot
from pycram.robot_plans import ParkArmsActionDescription, NavigateActionDescription

try:
    from pycram.external_interfaces.robokudo import send_query

    _ROBOKUDO_AVAILABLE = True
except ModuleNotFoundError:
    logging.warning(
        "robokudo_msgs not found – waving-human queries will use dummy data."
    )
    send_query = None
    _ROBOKUDO_AVAILABLE = False

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

# Configuration
SIMULATED: bool = True
MIN_DISTANCE_M: float = 1.0

_DUMMY = {
    "frame_id": "map",
    "position": [3.8683114051818848, 5.459158897399902, 0.0],
    "orientation": [0.0, 0.0, 0.04904329912700753, 0.9987966533838301],
}


def _extract_pose_fields(result) -> Optional[dict]:
    """Extract raw pose numbers from a RoboKudo Query.Result."""
    try:
        ros_ps = result.res[0].pose[0]
    except (AttributeError, IndexError, TypeError):
        return None
    return {
        "frame_id": ros_ps.header.frame_id,
        "position": [
            ros_ps.pose.position.x,
            ros_ps.pose.position.y,
            ros_ps.pose.position.z,
        ],
        "orientation": [
            ros_ps.pose.orientation.x,
            ros_ps.pose.orientation.y,
            ros_ps.pose.orientation.z,
            ros_ps.pose.orientation.w,
        ],
    }


def wait_for_waving_human(
    retry_interval: float = 1.0,
    timeout: Optional[float] = None,
) -> dict:
    """Poll RoboKudo until a waving human is found.

    Falls back to dummy data when RoboKudo is unavailable.
    """
    if not _ROBOKUDO_AVAILABLE:
        logger.warning("RoboKudo not available – using dummy waving-human pose.")
        return _DUMMY

    deadline = time.monotonic() + timeout if timeout is not None else None
    attempt = 0
    while True:
        attempt += 1
        logger.info("Waving query attempt %d …", attempt)
        result = send_query(obj_type="human", attributes=["waving"])
        if result is None:
            logger.warning(
                "RoboKudo action server not available – falling back to dummy data."
            )
            return _DUMMY
        data = _extract_pose_fields(result)
        if data is not None:
            logger.info("Waving human found after %d attempt(s).", attempt)
            return data
        logger.warning("RoboKudo result has no pose – retrying …")

        if deadline is not None and time.monotonic() >= deadline:
            logger.error("Timed out waiting for waving human.")
            return _DUMMY
        time.sleep(retry_interval)


if not rclpy.ok():
    rclpy.init()

# World setup
if SIMULATED:
    from demos.pycram_suturo_demos.helper_methods_and_useful_classes.robot_setup import (
        robot_setup,
    )

    setup_result = robot_setup(simulation=True, with_objects=False)
    world, robot_view, context = (
        setup_result.world,
        setup_result.robot_view,
        setup_result.context,
    )
else:
    from demos.pycram_suturo_demos.pycram_basic_hsr_demos.start_up import (
        setup_hsrb_context,
    )

    _node, world, robot_view, context = setup_hsrb_context()


def get_robot_pose() -> PoseStamped:
    """Return the current robot base pose."""
    return PoseStamped.from_spatial_type(robot_view.root.global_pose)


# Detect waving human
human_data = wait_for_waving_human(retry_interval=1.0)

pos = human_data["position"]
ori = human_data["orientation"]
logger.info("Waving human detected at pos=%s", pos)


# Compute navigation target (min distance from human)
robot_pose = get_robot_pose()

human_pose_clean = PoseStamped.from_list(
    position=pos,
    orientation=ori,
    frame=world.root,
)

target_ros = min_distance_2_human(
    human_pose=human_pose_clean.ros_message(),
    robot_pose=robot_pose.ros_message(),
    min_distance=MIN_DISTANCE_M,
)

# Navigation target: position at min_distance from human, facing the human.
nav_target = PoseStamped.from_list(
    position=[
        target_ros.pose.position.x,
        target_ros.pose.position.y,
        target_ros.pose.position.z,
    ],
    orientation=[
        target_ros.pose.orientation.x,
        target_ros.pose.orientation.y,
        target_ros.pose.orientation.z,
        target_ros.pose.orientation.w,
    ],
    frame=world.root,
)

logger.info(
    "Nav target: x=%.3f  y=%.3f",
    float(nav_target.position.x),
    float(nav_target.position.y),
)

# Execute plan
plan = SequentialPlan(
    context,
    ParkArmsActionDescription(Arms.BOTH),
    NavigateActionDescription(target_location=nav_target),
)

executor = simulated_robot if SIMULATED else real_robot

with executor:
    plan.perform()

# Log where the robot actually stopped
final_pose = get_robot_pose()
final_pos = [
    float(final_pose.position.x),
    float(final_pose.position.y),
    float(final_pose.position.z),
]
goal_pos = [
    float(nav_target.position.x),
    float(nav_target.position.y),
    float(nav_target.position.z),
]

dist_to_goal = np.linalg.norm(np.array(final_pos[:2]) - np.array(goal_pos[:2]))
dist_to_human = np.linalg.norm(np.array(final_pos[:2]) - np.array(pos[:2]))

print("=== Navigation result ===")
print(f"  Robot stopped at:  x={final_pos[0]:.3f}  y={final_pos[1]:.3f}")
print(f"  Goal was at:       x={goal_pos[0]:.3f}  y={goal_pos[1]:.3f}")
print(f"  Human is at:       x={pos[0]:.3f}  y={pos[1]:.3f}")
print(f"  Distance to goal:  {dist_to_goal:.3f} m")
print(f"  Distance to human: {dist_to_human:.3f} m (requested: {MIN_DISTANCE_M:.1f} m)")

print("Done – robot reached the target.")
