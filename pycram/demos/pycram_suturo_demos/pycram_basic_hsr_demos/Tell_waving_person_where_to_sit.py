from enum import Enum
from typing import Optional

from demos.pycram_suturo_demos.helper_methods_and_useful_classes.waving_detection import (
    ContinuousWavingDetector,
)
from pycram.external_interfaces import nav2_move
from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import PoseStamped
from demos.pycram_suturo_demos.pycram_basic_hsr_demos.start_up import setup_hsrb_context

from pycram.external_interfaces.nav2_move import min_distance_2_position
from pycram.external_interfaces.robokudo import shutdown_robokudo_interface
from pycram.external_interfaces.robokudo_ros1 import query_specific_region
from pycram.ros_utils.text_to_image import TextToImagePublisher
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot
from pycram.robot_plans import ParkArmsActionDescription, LookAtActionDescription

# rclpy.init()
rclpy_node, world, robot_view, context = setup_hsrb_context()

camera_frame = robot_view.get_default_camera().root
base_frame = world.get_body_by_name("base_link")

MIN_DISTANCE_M: float = 0.5
WAVING_TIMEOUT_PER_DIRECTION: float = 10.0


class Direction(Enum):
    LEFT = [0.1, 1, 0.75]
    RIGHT = [0.1, -1, 0.75]
    BACK = [-1, 0, 0.75]
    FRONT = [1, 0, 0.75]
    FRONT_DOWN = [1, 0, 0.5]


def get_robot_pose() -> PoseStamped:
    return PoseStamped.from_spatial_type(robot_view.root.global_pose)


def park_arms():
    SequentialPlan(
        context,
        ParkArmsActionDescription(Arms.BOTH),
    ).perform()


def look_in_direction(direction: Direction):
    SequentialPlan(
        context,
        ParkArmsActionDescription(Arms.BOTH),
        LookAtActionDescription(
            [PoseStamped.from_list(direction.value, frame=base_frame)]
        ),
    ).perform()


def drive_to_pose(target_pose: PoseStamped):
    robot = get_robot_pose()
    nav_target = min_distance_2_position(
        human_pose=target_pose.ros_message(),
        robot_pose=robot.ros_message(),
        min_distance=MIN_DISTANCE_M,
    )
    park_arms()
    nav2_move.start_nav_to_pose(nav_target)


def scan_for_waving_human() -> Optional[PoseStamped]:
    detector = ContinuousWavingDetector(retry_interval=1.0)

    human = detector.wait_for_waving_human(timeout=WAVING_TIMEOUT_PER_DIRECTION)

    for direction in [Direction.LEFT, Direction.RIGHT, Direction.BACK, Direction.FRONT]:
        if human is not None:
            break
        look_in_direction(direction)
        human = detector.wait_for_waving_human(timeout=WAVING_TIMEOUT_PER_DIRECTION)

    return human


def find_free_seat() -> str:
    look_in_direction(Direction.FRONT_DOWN)
    return query_specific_region("sofa")


with real_robot:
    text_pub = TextToImagePublisher()

    # 1. Scan for a waving human
    human = scan_for_waving_human()
    if human is None:
        text_pub.publish_text("No waving human found, giving up.")
        shutdown_robokudo_interface()
        exit(1)

    human_pose = PoseStamped.from_list(
        position=human.position.to_list(),
        orientation=human.orientation.to_list(),
        frame=world.root,
    )

    # 2. Drive to the human
    drive_to_pose(human_pose)

    # 3. Drive to sofa
    sofa_pose = PoseStamped.from_list(
        position=[3.60, 1.20, 0.0],
        orientation=[0.0, 0.0, 0.0, 1.0],
        frame=world.root,
    )
    drive_to_pose(sofa_pose)

    # 4. Find a free seat
    result = find_free_seat()

    # 5. Drive back to the human
    drive_to_pose(human_pose)

    # 6. Tell the human where to sit
    text_pub.publish_text(f"Free seat at {result}")

    shutdown_robokudo_interface()
