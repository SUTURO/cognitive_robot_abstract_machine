import logging
import os
from enum import Enum
from time import sleep
from typing import Optional

import semantic_digital_twin
from pycram.external_interfaces import nav2_move
from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import PoseStamped
from pycram_suturo_demos.pycram_basic_hsr_demos.start_up import setup_hsrb_context
from pycram.external_interfaces.nav2_move import buffer_in_front_of, change_orientation
from pycram.external_interfaces.robokudo import shutdown_robokudo_interface, send_query
from pycram.ros_utils.text_to_image import TextToImagePublisher
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot
from pycram.robot_plans import (
    ParkArmsActionDescription,
    LookAtActionDescription,
    MoveTorsoActionDescription,
)
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.datastructures.definitions import TorsoState

logger = logging.getLogger(__name__)
logging.getLogger(semantic_digital_twin.world.__name__).setLevel(logging.WARN)

rclpy_node, world, robot_view, context = setup_hsrb_context()


class Direction(Enum):
    FRONT = [1, 0, 1]
    FRONT_DOWN = [1, 0, 0.25]


def park_arms():
    SequentialPlan(
        context,
        ParkArmsActionDescription(Arms.BOTH),
    ).perform()


def get_robot_pose() -> PoseStamped:
    return PoseStamped.from_spatial_type(robot_view.root.global_pose)


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


move_torso_low1 = SequentialPlan(
    context,
    ParkArmsActionDescription(Arms.BOTH),
    MoveTorsoActionDescription(TorsoState.LOW),
)
move_torso_low2 = SequentialPlan(
    context,
    ParkArmsActionDescription(Arms.BOTH),
    MoveTorsoActionDescription(TorsoState.LOW),
)


def move_torso_mid(direction: Direction):
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
        MoveTorsoActionDescription(TorsoState.MID),
        LookAtActionDescription([look_at_pose_in_map.to_pose()]),
    ).perform()


def print_result(result):
    print(result)
    for r in result:
        print(r.type)
        text_pub.publish_text(f"Seen {r.type}.")
        sleep(2)


with real_robot:
    text_pub = TextToImagePublisher()

    start_pose = get_robot_pose()

    # Driving to shelf (Koordinaten anpassen!)
    text_pub.publish_text("Driving to shelf.")
    shelf_pose = PoseStamped.from_list(
        position=[3.572, 5.334, 0.0],
        orientation=[0.0, 0.0, 0.04904329912700753, 0.9987966533838301],
        frame=world.root,
    )
    park_arms()
    goal = shelf_pose.ros_message()
    nav2_move.start_nav_to_pose(goal)

    # Looking at shelf
    text_pub.publish_text("Looking at shelf.")
    move_torso_low1.perform()
    look_in_direction(Direction.FRONT_DOWN)
    text_pub.publish_text("Looking at low shelf.")
    o_test = send_query(obj_type="object")
    sleep(5)
    result_low = send_query(obj_type="object")
    print(f"Low: {result_low}")
    print("Looking at shelf FRONT.")
    move_torso_mid(Direction.FRONT)
    text_pub.publish_text("Looking at high shelf.")
    o_test2 = send_query(obj_type="object")
    sleep(5)
    result_high = send_query(obj_type="object")
    print(f"High: {result_high}")
    move_torso_low2.perform()

    # Driving back
    text_pub.publish_text("Driving back to person.")
    final_move = start_pose.ros_message()
    nav2_move.start_nav_to_pose(final_move)

    # Reporting what is on the shelf
    if result_low is None and result_high is None:
        text_pub.publish_text("No objects seen.")
    elif len(result_low.res) == 0 and len(result_high.res) == 0:
        text_pub.publish_text("No objects seen.")
    elif result_low is None or len(result_low.res) == 0:
        print_result(result_high.res)
    elif result_high is None or len(result_high.res) == 0:
        print_result(result_low.res)
    else:
        print(result_low.res)
        print(result_high.res)
        print_result(result_low.res)
        print_result(result_high.res)
        # result_comb = result_low.res.extend(result_high.res)
        # print(result_comb)
        # print_result(result_comb)
