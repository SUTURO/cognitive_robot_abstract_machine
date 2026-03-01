"""
roboter_to_human.py – Detect a waving human and drive toward them,
keeping a minimum stand-off distance.

Set ``SIMULATED`` to switch between simulation and real robot.
"""

import logging

import numpy as np
import rclpy

from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import PoseStamped
from pycram.external_interfaces.nav2_move import min_distance_2_human
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot, simulated_robot
from pycram.robot_plans import ParkArmsActionDescription, NavigateActionDescription

from waving_continuous import ContinuousWavingDetector

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

# Configuration
SIMULATED: bool = True
MIN_DISTANCE_M: float = 1.0

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
detector = ContinuousWavingDetector(retry_interval=1.0)
human_pose = detector.wait_for_waving_human()

if human_pose is None:
    logger.error("No waving human detected – aborting.")
    raise SystemExit(1)

logger.info("Waving human detected at %s", human_pose)

# Compute navigation target (min distance from human)
robot_pose = get_robot_pose()

# Rebuild poses with from_list so the header stamp is a proper datetime.
human_pose_clean = PoseStamped.from_list(
    position=human_pose.position.to_list(),
    orientation=human_pose.orientation.to_list(),
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
human_pos = human_pose.position.to_list()

dist_to_goal = np.linalg.norm(np.array(final_pos[:2]) - np.array(goal_pos[:2]))
dist_to_human = np.linalg.norm(
    np.array(final_pos[:2]) - np.array([float(v) for v in human_pos[:2]])
)

print("=== Navigation result ===")
print(f"  Robot stopped at:  x={final_pos[0]:.3f}  y={final_pos[1]:.3f}")
print(f"  Goal was at:       x={goal_pos[0]:.3f}  y={goal_pos[1]:.3f}")
print(f"  Human is at:       x={float(human_pos[0]):.3f}  y={float(human_pos[1]):.3f}")
print(f"  Distance to goal:  {dist_to_goal:.3f} m")
print(f"  Distance to human: {dist_to_human:.3f} m (requested: {MIN_DISTANCE_M:.1f} m)")
print("Done – robot reached the target.")
