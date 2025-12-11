"""This is a demo for Milestone 1"""

from enum import Enum

import rclpy
from geometry_msgs.msg import PoseStamped as ROSPoseStamped

from pycram.datastructures.pose import Pose
from pycram.external_interfaces import nav2_move
from pycram.ros_utils.text_to_image import TextToImagePublisher


class NavigatePositions(Enum):
    IN_FRONT_DISHWASHER = Pose.from_list(
        position=[2.322282075881958, -1.2466719150543213, 0.0],
        orientation=[0.0, 0.0, -0.6304255522851391, 0.7762497169248935],
    )
    SOFA_FACING = Pose.from_list(
        position=[3.325042486190796, 2.6914186477661133, 0.0],
        orientation=[0.0, 0.0, -0.6548372471824481, 0.75576992511115],
    )
    CABINET = Pose.from_list(
        position=[3.8683114051818848, 5.459158897399902, 0.0],
        orientation=[0.0, 0.0, 0.04904329912700753, 0.9987966533838301],
    )


def real_demo():
    # rclpy.init()
    text_pub = TextToImagePublisher()
    for pos in NavigatePositions:
        goal = ROSPoseStamped()
        goal.header.frame_id = "map"
        goal.pose = pos.value.ros_message()
        text_pub.publish_text(f"Moving to: {pos.name}")
        nav2_move.start_nav_to_pose(goal)
    # rclpy.shutdown()


real_demo()
