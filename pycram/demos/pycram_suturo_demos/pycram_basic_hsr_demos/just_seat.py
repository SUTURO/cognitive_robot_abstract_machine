import logging
import os
from enum import Enum
from typing import Optional
from time import sleep

import pycram.external_interfaces.robokudo
import semantic_digital_twin
from pycram_suturo_demos.helper_methods_and_useful_classes.waving_detection import (
    ContinuousWavingDetector,
)
from pycram.external_interfaces import nav2_move
from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import PoseStamped
from pycram_suturo_demos.pycram_basic_hsr_demos.A_start_up import setup_hsrb_context
from pycram.external_interfaces.nav2_move import buffer_in_front_of
from pycram.external_interfaces.robokudo import shutdown_robokudo_interface
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

MIN_DISTANCE_M: float = 0
WAVING_TIMEOUT_PER_DIRECTION: float = 8.0
ORIENTATION_SWITCH: bool = True


class Direction(Enum):
    LEFT = [0.1, 1, 1]
    RIGHT = [0.1, -1, 1]
    BACK = [-1, 0, 1]
    FRONT = [1, 0, 1]
    FRONT_DOWN = [1, 0, 0.5]


def get_robot_pose() -> PoseStamped:
    return PoseStamped.from_spatial_type(robot_view.root.global_pose)


def transform_perception_to_map(perception_pose: PoseStamped) -> PoseStamped:
    pose_in_camera = HomogeneousTransformationMatrix.from_xyz_quaternion(
        pos_x=float(perception_pose.position.x),
        pos_y=float(perception_pose.position.y),
        pos_z=float(perception_pose.position.z),
        quat_x=float(perception_pose.orientation.x),
        quat_y=float(perception_pose.orientation.y),
        quat_z=float(perception_pose.orientation.z),
        quat_w=float(perception_pose.orientation.w),
        reference_frame=world.get_body_by_name("head_rgbd_sensor_link"),
    )

    pose_in_map = world.transform(pose_in_camera, world.root)
    result = PoseStamped.from_spatial_type(pose_in_map)

    result.position.z = 0.0
    if ORIENTATION_SWITCH:
        head_pan = world.get_body_by_name("head_pan_link")
        head_pan_pose = PoseStamped.from_spatial_type(head_pan.global_pose)
        result.orientation = head_pan_pose.orientation
    logger.info(
        f"Transformierte Pose in map: Position=({result.position.x:.3f}, {result.position.y:.3f}, {result.position.z:.3f}), "
        f"Orientation=({result.orientation.x:.3f}, {result.orientation.y:.3f}, {result.orientation.z:.3f}, {result.orientation.w:.3f})"
    )
    return result


def park_arms():
    SequentialPlan(
        context,
        ParkArmsActionDescription(Arms.BOTH),
    ).perform()


def look_in_direction(direction: Direction):
    look_at_pose = HomogeneousTransformationMatrix.from_xyz_rpy(
        x=direction.value[0],
        y=direction.value[1],
        z=direction.value[2],
        reference_frame=robot_view.root,
    )
    look_at_pose_in_map = world.transform(look_at_pose, world.root)
    SequentialPlan(
        context,
        ParkArmsActionDescription(Arms.BOTH),
        LookAtActionDescription([look_at_pose_in_map.to_pose()]),
    ).perform()


def drive_to_pose(target_pose: PoseStamped):

    # nav_target = buffer_in_front_of(
    #     target_pose,
    #     min_distance=MIN_DISTANCE_M,
    # )
    # nav_target = target_pose

    park_arms()
    nav2_move.start_nav_to_pose(target_pose)


def scan_for_waving_human() -> Optional[PoseStamped]:
    detector = ContinuousWavingDetector(retry_interval=1.0)

    s_human = detector.wait_for_waving_human(timeout=WAVING_TIMEOUT_PER_DIRECTION)

    for direction in [Direction.LEFT, Direction.RIGHT, Direction.BACK, Direction.FRONT]:
        if s_human is not None:
            break
        look_in_direction(direction)
        s_human = detector.wait_for_waving_human(timeout=WAVING_TIMEOUT_PER_DIRECTION)
    look_in_direction(Direction.FRONT)
    return s_human


def find_free_seat() -> str:
    look_in_direction(Direction.FRONT_DOWN)
    return pycram.external_interfaces.robokudo.query_specific_region("sofa")


with real_robot:
    print(get_robot_pose().orientation)
    os.environ["ROS_PYTHON_CHECK_FIELDS"] = "1"
    # text_pub = TextToImagePublisher()
    # 1.5 , 2.26
    # 1. Scan for a waving human
    # text_pub.publish_text("Looking for a waving human...")
    # look_in_direction(Direction.FRONT)
    # human = scan_for_waving_human()
    # if human is None:
    #     text_pub.publish_text("No waving human found, giving up.")
    #     shutdown_robokudo_interface()
    #     exit(1)
    # human_pose = transform_perception_to_map(human)
    # text_pub.publish_text(f"Found human: {human.pose.position}")
    # # 2. Drive to the human
    # human_goal = human_pose.ros_message()
    # text_pub.publish_text(f"Driving to human: {human_goal.pose.position}")
    # drive_to_pose(human_goal)
    # sleep(2)

    # 3. Drive to sofa
    # sofa_pose = PoseStamped.from_list(
    #     position=[3.61, 3.4, 0.0],
    #     orientation=[0.0, 0.0, -0.7071, 0.7071],
    #     frame=world.root,
    # )
    # goal = sofa_pose.ros_message()
    # text_pub.publish_text(f"Driving to sofa: {goal.pose.position}")
    # nav2_move.start_nav_to_pose(goal)
    print(get_robot_pose().position)
    # 4. Find a free seat
    # test = pycram.external_interfaces.robokudo.send_query()
    # sleep(2)
    # result = find_free_seat()
    # text_pub.publish_text(f"Got a result")

    # 5. Drive back to the human
    # text_pub.publish_text(f"Driving back to human: {human_goal.pose.position}")
    # drive_to_pose(human_goal)

    # 6. Tell the human where to sit
    # if len(result.res) == 0:
    #     print(f"Aborted")
    # else:
    #     right_seat = result.res[0].attribute[0]
    #     list_right = right_seat.split(",")
    #     left_seat = result.res[0].attribute[1]
    #     list_left = left_seat.split(",")
    #     print(list_right)
    #     print(list_left)
    #     if list_right[1] == " False" and list_left[1] == " False":
    #         text_pub.publish_text(f"Both seats free")
    #         print(f"Both seats free")
    #     elif list_right[1] == " True" and list_left[1] == " True":
    #         text_pub.publish_text(f"No seats free")
    #         print(f"No seats free")
    #     elif list_right[1] == " False" and list_left[1] == " True":
    #         text_pub.publish_text(f"Right seats free")
    #         print(f"Right seat free")
    #     else:
    #         text_pub.publish_text(f"Left seats free")
    #         print(f"Left seat free")

    shutdown_robokudo_interface()
