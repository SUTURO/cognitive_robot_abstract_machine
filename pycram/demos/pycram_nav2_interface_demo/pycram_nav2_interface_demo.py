import threading
from time import sleep

import rclpy

from pycram.external_interfaces import nav2_move
from geometry_msgs.msg import Pose, Point, PoseStamped, Quaternion

# TODO: real pycram demo
# Demo testet with the following running:
# ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
# set estimated Pose for robot first
#
# The demo sets a first goal, cancels it with a new goal and then cancel the second goal as well to show functionality

sofa_back = Pose(position=Point(y=0.7), orientation=Quaternion(z=0.707, w=0.707))
dishwasher = Pose(
    position=Point(x=-1.7682, y=-2.3339), orientation=Quaternion(z=-0.8213, w=0.5704)
)


def start_nav(pos: Pose):
    pos_stamped = PoseStamped()
    pos_stamped.header.frame_id = "map"
    pos_stamped.pose = pos
    nav2_move.start_nav_to_pose(pos_stamped)


if __name__ == "__main__":
    rclpy.init()

    # Initialize threads
    first_goal = threading.Thread(target=start_nav, args=(sofa_back,))
    second_goal = threading.Thread(target=start_nav, args=(dishwasher,))
    cancel_goal = threading.Thread(target=nav2_move.cancel_current_goal)

    # Start actions for demo
    first_goal.start()
    # sleep(5)
    # second_goal.start()
    # sleep(3)
    # cancel_goal.start()

    # Wait till done
    first_goal.join()
    # second_goal.join()
    # cancel_goal.join()

    # Not really needed, but making sure
    nav2_move.shutdown_nav_interface()

    rclpy.shutdown()
