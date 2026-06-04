import logging
from enum import Enum
from time import sleep

import semantic_digital_twin
from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import PoseStamped
from pycram.external_interfaces import nav2_move
from pycram.external_interfaces.nlp_interface import TalkingNode
from pycram.external_interfaces.robokudo import send_query
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot
from pycram.robot_plans import (
    LookAtActionDescription,
    MoveTorsoActionDescription,
    ParkArmsActionDescription,
)
from pycram.ros_utils.text_to_image import TextToImagePublisher
from pycram_suturo_demos.pycram_basic_hsr_demos.A_start_up import setup_hsrb_context
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix

# Configure logging
logger = logging.getLogger(__name__)
logging.getLogger(semantic_digital_twin.world.__name__).setLevel(logging.WARN)

# Initialize robot context
rclpy_node, world, robot_view, context = setup_hsrb_context()


class Direction(Enum):
    """Enumeration for different look directions."""

    FRONT = [1, 0, 1]
    FRONT_DOWN = [1, 0, 0.25]


def park_arms():
    """Parks both arms of the robot."""
    SequentialPlan(
        context,
        ParkArmsActionDescription(Arms.BOTH),
    ).perform()


def get_robot_pose() -> PoseStamped:
    """
    Retrieves the current pose of the robot.

    Returns:
        PoseStamped: The robot's current pose in the world frame.
    """
    return PoseStamped.from_spatial_type(robot_view.root.global_pose)


def look_in_direction(direction: Direction):
    """
    Makes the robot look in a specified direction.

    Args:
        direction (Direction): The direction to look at.
    """
    look_at_pose = HomogeneousTransformationMatrix.from_xyz_rpy(
        x=direction.value[0],
        y=direction.value[1],
        z=direction.value[2],
        reference_frame=robot_view.root,
    )
    look_at_pose_in_map = world.transform(look_at_pose, world.root)
    SequentialPlan(
        context,
        LookAtActionDescription([look_at_pose_in_map.to_pose()]),
    ).perform()


def move_torso_low():
    """Moves the robot's torso to the low position."""
    SequentialPlan(
        context,
        MoveTorsoActionDescription(TorsoState.LOW),
    ).perform()


def move_torso_mid(direction: Direction):
    """
    Moves the robot's torso to the middle position and looks in a specified direction.

    Args:
        direction (Direction): The direction to look at after moving the torso.
    """
    look_at_pose = HomogeneousTransformationMatrix.from_xyz_rpy(
        x=direction.value[0],
        y=direction.value[1],
        z=direction.value[2],
        reference_frame=robot_view.root,
    )
    look_at_pose_in_map = world.transform(look_at_pose, world.root)
    SequentialPlan(
        context,
        MoveTorsoActionDescription(TorsoState.MID),
        LookAtActionDescription([look_at_pose_in_map.to_pose()]),
    ).perform()


def print_result(result, text_pub: TextToImagePublisher, tts: TalkingNode):
    """
    Prints and announces the detected objects.

    Args:
        result: The list of detected objects from RoboKudo.
        text_pub (TextToImagePublisher): Publisher for displaying text on an image.
        tts (TalkingNode): Node for text-to-speech announcements.
    """
    print(result)
    for r in result:
        print(r.type)
        text_pub.publish_text(f"Seen {r.type}.")
        tts.pub(f"Seen {r.type}")
        sleep(2)


def main():
    """
    Main function to execute the "tell me what is on the shelf" demo.
    The robot drives to a shelf, scans it at two different heights,
    reports the objects it finds, and returns to its starting position.
    """
    with real_robot:
        text_pub = TextToImagePublisher()
        tts = TalkingNode()
        start_pose = get_robot_pose()

        # Driving to shelf
        text_pub.publish_text("Driving to shelf.")
        tts.pub("Driving to shelf")
        shelf_pose = PoseStamped.from_list(
            position=[1.4, 3.4, 0.0],
            orientation=[0.0, 0.0, -1, 0.0],
            frame=world.root,
        )
        park_arms()
        nav2_move.start_nav_to_pose(shelf_pose.ros_message())

        # Looking at shelf
        text_pub.publish_text("Looking at shelf.")
        tts.pub("Looking at shelf")

        # Scan low shelf
        move_torso_low()
        look_in_direction(Direction.FRONT_DOWN)
        text_pub.publish_text("Looking at low shelf.")
        tts.pub("Looking at low shelf")
        sleep(1)  # Wait for perception
        result_low = send_query(obj_type="object")
        print(f"Low: {result_low}")

        # Scan high shelf
        move_torso_mid(Direction.FRONT)
        text_pub.publish_text("Looking at high shelf.")
        tts.pub("Looking at high shelf")
        sleep(1)  # Wait for perception
        result_high = send_query(obj_type="object")
        print(f"High: {result_high}")
        move_torso_low()

        # Driving back
        text_pub.publish_text("Driving back to person.")
        tts.pub("Driving back to person")
        nav2_move.start_nav_to_pose(start_pose.ros_message())

        # Reporting what is on the shelf
        all_results = []
        if result_low and result_low.res:
            all_results.extend(result_low.res)
        if result_high and result_high.res:
            all_results.extend(result_high.res)

        if not all_results:
            text_pub.publish_text("No objects seen.")
            tts.pub("No objects seen")
        else:
            print_result(all_results, text_pub, tts)


if __name__ == "__main__":
    main()
