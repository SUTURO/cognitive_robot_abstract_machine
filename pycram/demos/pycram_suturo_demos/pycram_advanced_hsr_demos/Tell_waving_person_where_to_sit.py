import logging
import os
from enum import Enum
from typing import Optional
from time import sleep
import pycram.external_interfaces.robokudo
import semantic_digital_twin
from pycram_suturo_demos.helper_methods_and_useful_classes.nlp_human_robot_interaction import (
    TalkingNode,
)
from pycram_suturo_demos.helper_methods_and_useful_classes.waving_detection import (
    ContinuousWavingDetector,
)
from pycram.external_interfaces import nav2_move, robokudo
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
from pycram_suturo_demos.pycram_basic_hsr_demos.dialog_with_human_demo import (
    main as dialog_with_human_demo_main,
)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TtsPublisher:
    def __init__(self, node_name="tts_text_publisher", topic_name="/tts_text"):
        self.node = Node(node_name)
        self.publisher = self.node.create_publisher(String, topic_name, 10)

    def pub(self, text: str):
        msg = String()
        msg.data = text
        self.publisher.pub(msg)
        self.node.get_logger().info(f"Published: '{text}'")
        rclpy.spin_once(self.node, timeout_sec=0.1)

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()


logging.getLogger(semantic_digital_twin.world.__name__).setLevel(logging.WARN)

rclpy_node, world, robot_view, context = setup_hsrb_context()
camera_frame = robot_view.get_default_camera().root
base_frame = world.get_body_by_name("base_link")

MIN_DISTANCE_M: float = 0.8
WAVING_TIMEOUT_PER_DIRECTION: float = 10.0
BUFFER_SWITCH = False


class Direction(Enum):
    LEFT = [0.1, 1, 1]
    RIGHT = [0.1, -1, 1]
    BACK = [-1, 0, 1]
    FRONT = [1, 0, 1]
    FRONT_SOFA = [1, 0, 0.65]
    FRONT_DOWN = [1, 0, 0.5]


def get_sofa_pose() -> PoseStamped:
    sofa_pose = PoseStamped.from_list(
        position=[2.436592493983716, -1.263586281296266, 0.0],
        orientation=[0.0, 0.0, 0.2790542921412484, 0.7430916153276875],
        frame=world.root,
    )
    return sofa_pose


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
    head_pan = world.get_body_by_name("head_pan_link")
    head_pan_pose = PoseStamped.from_spatial_type(head_pan.global_pose)
    result.orientation = head_pan_pose.orientation
    print(
        f"Transformierte Pose in map: Position=({result.position.x:.3f}, {result.position.y:.3f}, {result.position.z:.3f})"
    )
    print(
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
        # ParkArmsActionDescription(Arms.BOTH),
        LookAtActionDescription([look_at_pose_in_map.to_pose()]),
    ).perform()


def drive_to_pose(target_pose: PoseStamped):
    goal_pose = target_pose
    if BUFFER_SWITCH:
        goal_pose = buffer_in_front_of(target_pose, MIN_DISTANCE_M)
    park_arms()
    nav2_move.start_nav_to_pose(goal_pose)


def scan_for_waving_human() -> Optional[PoseStamped]:
    detector = ContinuousWavingDetector(retry_interval=1.0)
    s_human = detector.wait_for_waving_human(timeout=WAVING_TIMEOUT_PER_DIRECTION)
    for direction in [
        Direction.FRONT,
        Direction.RIGHT,
        Direction.BACK,
        Direction.FRONT,
    ]:
        if s_human is not None:
            break
        look_in_direction(direction)
        s_human = detector.wait_for_waving_human(timeout=WAVING_TIMEOUT_PER_DIRECTION)
    look_in_direction(Direction.FRONT)
    return s_human


def find_free_seat() -> str:
    look_in_direction(Direction.FRONT_SOFA)
    return pycram.external_interfaces.robokudo.query_specific_region("sofa")


def main():
    with real_robot:
        start_position = get_robot_pose()
        print(
            f"Robot Position-> X:{get_robot_pose().position.x} Y:{get_robot_pose().position.y}"
        )
        os.environ["ROS_PYTHON_CHECK_FIELDS"] = "1"
        text_pub = TextToImagePublisher()
        tts = TalkingNode()

        # 1. Scan for a waving human
        park_arms()
        text_pub.publish_text("Looking for a waving human...")
        tts.pub("Looking for a waving human")
        test = robokudo.send_query()
        sleep(2)
        look_in_direction(Direction.LEFT)
        human = scan_for_waving_human()
        if human is None:
            text_pub.publish_text("No waving human found, giving up.")
            tts.pub("No waving human found, giving up.")
            shutdown_robokudo_interface()
            exit(1)

        human_pose = transform_perception_to_map(human)
        text_pub.publish_text(
            f"Found human:\n X:{human_pose.pose.position.x} \n Y:{human_pose.pose.position.y}"
        )
        tts.pub("Found Human")

        # 2. Drive to the human
        human_goal = human_pose.ros_message()
        text_pub.publish_text(
            f"Driving to human:\n X:{human_goal.pose.position.x} \n Y:{human_goal.pose.position.y}"
        )
        tts.pub("Driving to human")
        drive_to_pose(human_goal)

        # 2b. Dialog with the human
        # dialog_with_human_demo_main()
        tts.pub("I will now go look for a free seat for you!")
        sleep(3)

        # 3. Drive to sofa
        goal = get_sofa_pose().ros_message()
        text_pub.publish_text(
            f"Driving to sofa:\n X:{goal.pose.position.x} \n Y:{goal.pose.position.y}"
        )
        nav2_move.start_nav_to_pose(goal)
        print(
            f"Robot Position-> X:{get_robot_pose().position.x} Y:{get_robot_pose().position.y}"
        )

        # 4. Find a free seat
        test1 = robokudo.send_query()
        sleep(2)
        result = find_free_seat()
        text_pub.publish_text(f"Got Data")
        tts.pub("I got Data")

        # 5. Drive back to the human
        text_pub.publish_text(
            f"Driving back to human:\n X:{human_goal.pose.position.x} \n Y:{human_goal.pose.position.y}"
        )
        tts.pub("Driving back to human")
        drive_to_pose(human_goal)

        # 6. Tell the human where to sit
        if len(result.res) == 0:
            text_pub.publish_text(f"Something went wrong while finding seats.")
            tts.pub("Something went wrong while finding seats")
            print(f"Aborted")
        else:
            right_seat = result.res[0].attribute[0]
            list_right = right_seat.split(",")
            left_seat = result.res[0].attribute[1]
            list_left = left_seat.split(",")
            chair = result.res[0].attribute[2]
            list_chair = chair.split(",")
            print(list_right)
            print(list_left)
            print(list_chair)
            if (
                list_right[1] == " False"
                and list_left[1] == " False"
                and list_chair[1] == " False"
            ):
                text_pub.publish_text(f"Sofa is free and also chair")
                tts.pub("All seats are free")
            elif (
                list_right[1] == " True"
                and list_left[1] == " True"
                and list_chair[1] == " True"
            ):
                text_pub.publish_text(f"No seats free")
                tts.pub("No seats free")
            elif (
                list_right[1] == " False"
                and list_left[1] == " True"
                and list_chair[1] == " True"
            ):
                text_pub.publish_text(f"Right seats free")
                tts.pub("Right seats free")
            elif (
                list_right[1] == " True"
                and list_left[1] == " False"
                and list_chair[1] == " True"
            ):
                text_pub.publish_text(f"Left seats free")
                tts.pub("Left seats free")
            elif (
                list_right[1] == " True"
                and list_left[1] == " True"
                and list_chair[1] == " False"
            ):
                text_pub.publish_text(f"Chair is free")
                tts.pub("Chair is free")
            elif (
                list_right[1] == " False"
                and list_left[1] == " False"
                and list_chair[1] == " True"
            ):
                text_pub.publish_text(f"Right and left seats are free")
                tts.pub("Right and left seats are free")
            elif (
                list_right[1] == " False"
                and list_left[1] == " True"
                and list_chair[1] == " False"
            ):
                text_pub.publish_text(f"Right seat and chair are free")
                tts.pub("Right seat and chair are free")
            elif (
                list_right[1] == " True"
                and list_left[1] == " False"
                and list_chair[1] == " False"
            ):
                text_pub.publish_text(f"Left seat and chair are free")
                tts.pub("Left seat and chair are free")
            else:
                text_pub.publish_text(
                    f"Something went wrong while interpreting the data."
                )
                tts.pub("Something went wrong while interpreting the data")

        # 7. Drive back to the start position and shutdown
        tts.pub("I will now go back to the start position")
        nav2_move.start_nav_to_pose(start_position.ros_message())
        shutdown_robokudo_interface()


if __name__ == "__main__":
    main()
