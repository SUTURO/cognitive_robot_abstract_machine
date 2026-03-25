import logging
import os
from enum import Enum
from typing import Optional
from time import sleep
import pycram.external_interfaces.robokudo
import semantic_digital_twin
from demos.pycram_suturo_demos.helper_methods_and_useful_classes.waving_detection import (
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
from pycram.ros_utils.text_to_image import TextToImagePublisher
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix

# --- TtsPublisher class inlined from talking_demo.py ---
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError as e:
    raise ImportError(
        "ROS2 Python environment not found. Please source ROS2 and install rclpy and std_msgs."
    ) from e


class TtsPublisher:
    def __init__(self, node_name="tts_text_publisher", topic_name="/tts_text"):
        rclpy.init()
        self.node = Node(node_name)
        self.publisher = self.node.create_publisher(String, topic_name, 10)

    def publish(self, text: str):
        msg = String()
        msg.data = text
        self.publisher.publish(msg)
        self.node.get_logger().info(f"Published: '{text}'")
        rclpy.spin_once(self.node, timeout_sec=0.1)

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()


# --- End TtsPublisher class ---

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
    look_in_direction(Direction.FRONT_DOWN)
    return pycram.external_interfaces.robokudo.query_specific_region("sofa")


with real_robot:
    print(
        f"Robot Position-> X:{get_robot_pose().position.x} Y:{get_robot_pose().position.y}"
    )
    os.environ["ROS_PYTHON_CHECK_FIELDS"] = "1"
    text_pub = TextToImagePublisher()
    tts = TtsPublisher()
    # 1. Scan for a waving human
    text_pub.publish_text("Looking for a waving human...")
    tts.publish("Looking for a waving human")
    look_in_direction(Direction.LEFT)
    human = scan_for_waving_human()
    if human is None:
        text_pub.publish_text("No waving human found, giving up.")
        tts.publish("No waving human found, giving up.")
        shutdown_robokudo_interface()
        tts.shutdown()
        exit(1)
    human_pose = transform_perception_to_map(human)
    text_pub.publish_text(
        f"Found human:\n X:{human_pose.pose.position.x} \n Y:{human_pose.pose.position.y}"
    )
    tts.publish("Found Human")
    # 2. Drive to the human
    human_goal = human_pose.ros_message()
    text_pub.publish_text(
        f"Driving to human:\n X:{human_goal.pose.position.x } \n Y:{human_goal.pose.position.y}"
    )
    tts.publish("Driving to human")
    drive_to_pose(human_goal)
    text_pub.publish_text(f"Hello, I will now look for a free seat...")
    tts.publish("Hello, I will see if I can find a free seat for you")
    sleep(6)

    # 3. Drive to sofa
    sofa_pose = PoseStamped.from_list(
        position=[3.660223589941574, 3.507497103196295, 0.0],
        orientation=[0.0, 0.0, -0.6691896974921895, 0.7430916153276875],
        frame=world.root,
    )
    goal = sofa_pose.ros_message()
    text_pub.publish_text(
        f"Driving to sofa:\n X:{goal.pose.position.x} \n Y:{goal.pose.position.y}"
    )
    nav2_move.start_nav_to_pose(goal)
    print(
        f"Robot Position-> X:{get_robot_pose().position.x} Y:{get_robot_pose().position.y}"
    )
    # 4. Find a free seat
    result = find_free_seat()
    text_pub.publish_text(f"Got Data")
    tts.publish("I got Data")

    # 5. Drive back to the human
    text_pub.publish_text(
        f"Driving back to human:\n X:{human_goal.pose.position.x } \n Y:{human_goal.pose.position.y}"
    )
    drive_to_pose(human_goal)

    # 6. Tell the human where to sit
    if len(result.res) == 0:
        text_pub.publish_text(f"Something went wrong while finding seats.")
        tts.publish("Something went wrong while finding seats")
        print(f"Aborted")
    else:
        right_seat = result.res[0].attribute[0]
        list_right = right_seat.split(",")
        left_seat = result.res[0].attribute[1]
        list_left = left_seat.split(",")
        print(list_right)
        print(list_left)
        if list_right[1] == " False" and list_left[1] == " False":
            text_pub.publish_text(f"Both seats free")
            tts.publish("Both seats free")
            print(f"Both seats free")
        elif list_right[1] == " True" and list_left[1] == " True":
            text_pub.publish_text(f"No seats free")
            tts.publish("No seats free")
            print(f"No seats free")
        elif list_right[1] == " False" and list_left[1] == " True":
            text_pub.publish_text(f"Right seats free")
            tts.publish("Right seats free")
            print(f"Right seat free")
        else:
            text_pub.publish_text(f"Left seats free")
            tts.publish("Left seats free")
            print(f"Left seat free")

    shutdown_robokudo_interface()
    tts.shutdown()
