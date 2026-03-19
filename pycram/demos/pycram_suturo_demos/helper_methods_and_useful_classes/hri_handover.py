from __future__ import annotations


import rclpy

import nlp_human_robot_interaction as hri
from time import sleep

from pycram_suturo_demos.pycram_basic_hsr_demos.gripper_open_close_demo import (
    GripperActionClient,
)


def countdown(n, node: hri.TalkingNode):
    while n > 0:
        node.pub(str(n))
        sleep(1.5)
        n -= 1


def take_object_from_human():
    talk = hri.TalkingNode()
    gripper_open = 0.8
    gripper_close = -0.8

    grippy = GripperActionClient()
    grippy.send_goal(effort=gripper_open)

    talk.pub("Please put the object into my gripper. Closing gripper in ")
    countdown(5, talk)
    grippy.send_goal(effort=gripper_close)
    sleep(5)


def give_object_to_human():
    talk = hri.TalkingNode()
    gripper_open = 0.8

    grippy = GripperActionClient()

    talk.pub("Please take the object from my gripper. Opening gripper in ")
    countdown(5, talk)
    grippy.send_goal(effort=gripper_open)
    sleep(5)


def main():
    rclpy.init()
    take_object_from_human()
    # give_object_to_human()
    sleep(5)


if __name__ == "__main__":
    main()
