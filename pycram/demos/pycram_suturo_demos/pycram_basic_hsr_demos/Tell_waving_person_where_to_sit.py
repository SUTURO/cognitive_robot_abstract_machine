import logging
import os
from enum import Enum
from typing import Optional

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
from pycram.external_interfaces.robokudo_ros1 import query_specific_region
from pycram.ros_utils.text_to_image import TextToImagePublisher
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot
from pycram.robot_plans import ParkArmsActionDescription, LookAtActionDescription
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix

logger = logging.getLogger(__name__)
logging.getLogger(semantic_digital_twin.world.__name__).setLevel(logging.WARN)

rclpy_node, world, robot_view, context = setup_hsrb_context()

camera_frame = robot_view.get_default_camera().root
base_frame = world.get_body_by_name("base_link")

MIN_DISTANCE_M: float = 0.5
WAVING_TIMEOUT_PER_DIRECTION: float = 4.0


class Direction(Enum):
    LEFT = [0.1, 1, 0.75]
    RIGHT = [0.1, -1, 0.75]
    BACK = [-1, 0, 0.75]
    FRONT = [1, 0, 0.75]
    FRONT_DOWN = [1, 0, 0.5]


def get_robot_pose() -> PoseStamped:
    return PoseStamped.from_spatial_type(robot_view.root.global_pose)


def transform_perception_to_map(perception_pose: PoseStamped) -> PoseStamped:
    """Transform a pose from the perception (camera) frame into the map frame.

    Mirrors the ROS1 ``transform_camera_to_x`` logic:
      1. Transform the position from the camera frame into the map frame.
      2. Set z = 0 (floor level – the robot drives on the ground).
      3. Replace the orientation with the head_pan_link orientation so that
         the resulting pose faces the same direction as Toya's head.
    """
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


def look_in_direction(direction: HomogeneousTransformationMatrix):
    SequentialPlan(
        context,
        ParkArmsActionDescription(Arms.BOTH),
        LookAtActionDescription([direction.to_pose()]),
    ).perform()


def drive_to_pose(target_pose: PoseStamped):
    """Drive to a point MIN_DISTANCE_M in front of *target_pose*, then turn
    180° so the robot faces the object.

    The standoff point is computed purely from the object's orientation,
    not the robot's current position.
    """
    nav_target = buffer_in_front_of(
        target_pose.ros_message(),
        min_distance=MIN_DISTANCE_M,
    )

    park_arms()
    nav2_move.start_nav_to_pose(nav_target)

    arrived_pose = get_robot_pose()
    turned_pose = change_orientation(arrived_pose.ros_message())
    nav2_move.start_nav_to_pose(turned_pose)


def scan_for_waving_human() -> Optional[PoseStamped]:
    detector = ContinuousWavingDetector(retry_interval=1.0)

    s_human = detector.wait_for_waving_human(timeout=WAVING_TIMEOUT_PER_DIRECTION)
    print(s_human)

    for direction in [Direction.LEFT, Direction.RIGHT, Direction.BACK, Direction.FRONT]:
        if s_human is not None:
            break
        look_at_pose = HomogeneousTransformationMatrix.from_xyz_rpy(
            x=direction.value[0],
            y=direction.value[1],
            z=direction.value[2],
            reference_frame=robot_view.root,
        )
        look_at_pose_in_map = world.transform(look_at_pose, world.root)
        look_in_direction(look_at_pose_in_map)
        s_human = detector.wait_for_waving_human(timeout=WAVING_TIMEOUT_PER_DIRECTION)

    return s_human


def find_free_seat() -> str:
    look_in_direction(Direction.FRONT_DOWN)
    return query_specific_region("sofa")


with real_robot:
    os.environ["ROS_PYTHON_CHECK_FIELDS"] = "1"
    text_pub = TextToImagePublisher()

    # 1. Scan for a waving human
    front = Direction.FRONT
    look_at_pose = HomogeneousTransformationMatrix.from_xyz_rpy(
        x=front.value[0],
        y=front.value[1],
        z=front.value[2],
        reference_frame=robot_view.root,
    )
    look_at_pose_in_map = world.transform(look_at_pose, world.root)
    look_in_direction(look_at_pose_in_map)
    human = scan_for_waving_human()
    if human is None:
        text_pub.publish_text("No waving human found, giving up.")
        shutdown_robokudo_interface()
        exit(1)

    human_pose = transform_perception_to_map(human)

    # 2. Drive to the human
    # drive_to_pose(human_pose)
    # Moving to geometry_msgs.msg.PoseStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1772543508, nanosec=74506), frame_id='map'), pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=1.3, y=5.3, z=0.0), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.7474093186836598, w=0.6643638388299198)))'
    # 3. Drive to sofa
    sofa_pose = PoseStamped.from_list(
        position=[3.60, 1.20, 0.0],
        orientation=[0.0, 0.0, 0.0, 1.0],
        frame=world.root,
    )
    drive_to_pose(sofa_pose)

    # 4. Find a free seat
    # result = find_free_seat()

    # 5. Drive back to the human
    # drive_to_pose(human_pose)

    # 6. Tell the human where to sit
    # text_pub.publish_text(f"Free seat at {result}")

    shutdown_robokudo_interface()
