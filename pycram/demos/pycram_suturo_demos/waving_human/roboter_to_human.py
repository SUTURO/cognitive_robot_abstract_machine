"""
roboter_to_human.py
===================
Detects a waving human and drives the HSR-B toward that human while
keeping a configurable minimum distance using
:func:`nav2_move.min_distance_2_human`.

Set ``SIMULATED = True`` for the physics simulation or
``SIMULATED = False`` for the real robot.
"""

import logging

import rclpy

from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import PoseStamped
from pycram.external_interfaces.nav2_move import min_distance_2_human
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot, simulated_robot
from pycram.robot_plans import ParkArmsActionDescription, NavigateActionDescription

from waving_continuous import ContinuousWavingDetector

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
SIMULATED: bool = True
MIN_DISTANCE_M: float = 0.5

if not rclpy.ok():
    rclpy.init()

# ---------------------------------------------------------------------------
# World setup – depends on simulation vs. real robot
# ---------------------------------------------------------------------------
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
    """Return the current robot base pose from the world model."""
    return PoseStamped.from_spatial_type(robot_view.root.global_pose)


# ---------------------------------------------------------------------------
# Detect waving human
# ---------------------------------------------------------------------------

detector = ContinuousWavingDetector(retry_interval=1.0)
human_pose = detector.wait_for_waving_human()

if human_pose is None:
    logger.error("No waving human detected – aborting.")
    raise SystemExit(1)

logger.info("Waving human detected at %s", human_pose)

# ---------------------------------------------------------------------------
# Compute navigation target (min distance from human)
# ---------------------------------------------------------------------------

robot_pose = get_robot_pose()

# Rebuild human_pose with a proper datetime stamp so .ros_message() works
# (from_ros_message stores a ROS Time which Header.ros_message cannot handle)
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

# ---------------------------------------------------------------------------
# Build and execute plan
# ---------------------------------------------------------------------------

plan = SequentialPlan(
    context,
    ParkArmsActionDescription(Arms.BOTH),
    NavigateActionDescription(target_location=nav_target),
)

executor = simulated_robot if SIMULATED else real_robot

with executor:
    plan.perform()

logger.info("Done – robot reached the target.")
