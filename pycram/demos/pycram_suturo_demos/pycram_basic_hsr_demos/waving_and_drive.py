"""
Simple demo: Detect a waving human, transform the pose to map coordinates,
drive to a buffered standoff position in front of them.
"""

import logging
import os

import semantic_digital_twin
from pycram_suturo_demos.helper_methods_and_useful_classes.waving_detection import (
    ContinuousWavingDetector,
)
from pycram.external_interfaces import nav2_move
from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import PoseStamped
from pycram_suturo_demos.pycram_basic_hsr_demos.start_up import setup_hsrb_context
from pycram.external_interfaces.nav2_move import buffer_in_front_of, change_orientation
from pycram.external_interfaces.robokudo import shutdown_robokudo_interface
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot
from pycram.robot_plans import ParkArmsActionDescription
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix

logger = logging.getLogger(__name__)
logging.getLogger(semantic_digital_twin.world.__name__).setLevel(logging.WARN)


rclpy_node, world, robot_view, context = setup_hsrb_context()

MIN_DISTANCE_M: float = 0.8
WAVING_TIMEOUT: float = 5.0


def get_robot_pose() -> PoseStamped:
    return PoseStamped.from_spatial_type(robot_view.root.global_pose)


def transform_perception_to_map(perception_pose: PoseStamped) -> PoseStamped:
    """Camera frame → map frame (z=0, orientation from head_pan_link)."""
    frame_id = perception_pose.header.frame_id
    if isinstance(frame_id, str):
        reference_body = world.get_body_by_name(frame_id)
    else:
        reference_body = frame_id

    pose_in_camera = HomogeneousTransformationMatrix.from_xyz_quaternion(
        pos_x=float(perception_pose.position.x),
        pos_y=float(perception_pose.position.y),
        pos_z=float(perception_pose.position.z),
        quat_x=float(perception_pose.orientation.x),
        quat_y=float(perception_pose.orientation.y),
        quat_z=float(perception_pose.orientation.z),
        quat_w=float(perception_pose.orientation.w),
        reference_frame=reference_body,
    )

    pose_in_map = world.transform(pose_in_camera, world.root)
    result = PoseStamped.from_spatial_type(pose_in_map)

    result.position.z = 0.0

    head_pan = world.get_body_by_name("head_pan_link")
    head_pan_pose = PoseStamped.from_spatial_type(head_pan.global_pose)
    result.orientation = head_pan_pose.orientation

    return result


def park_arms():
    SequentialPlan(
        context,
        ParkArmsActionDescription(Arms.BOTH),
    ).perform()


def drive_to_pose(target_pose: PoseStamped):
    """Drive to a buffered standoff in front of the target, then turn 180° to face it."""
    nav_target = buffer_in_front_of(
        target_pose.ros_message(),
        min_distance=MIN_DISTANCE_M,
    )

    logger.info(f"Driving to standoff: {nav_target}")
    park_arms()
    nav2_move.start_nav_to_pose(nav_target)

    logger.info("Arrived at standoff, turning 180° to face the human...")
    arrived_pose = get_robot_pose()
    turned_pose = change_orientation(arrived_pose.ros_message())
    nav2_move.start_nav_to_pose(turned_pose)

    logger.info("Done — robot is now facing the human.")


with real_robot:
    os.environ["ROS_PYTHON_CHECK_FIELDS"] = "1"

    # 1. Detect waving human
    logger.info("Looking for a waving human...")
    detector = ContinuousWavingDetector(retry_interval=1.0)
    human = detector.wait_for_waving_human(timeout=WAVING_TIMEOUT)

    if human is None:
        logger.warning("No waving human found.")
        shutdown_robokudo_interface()
        exit(1)

    logger.info(f"Waving human detected (camera frame): {human}")

    # 2. Transform to map coordinates
    human_pose = transform_perception_to_map(human)
    logger.info(f"Human pose in map frame: {human_pose}")

    # 3. Drive to the human
    drive_to_pose(human_pose)

    shutdown_robokudo_interface()
