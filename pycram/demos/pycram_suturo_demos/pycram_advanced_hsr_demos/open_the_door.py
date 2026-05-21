from __future__ import annotations


import logging
import rclpy
from pycram.datastructures.pose import PoseStamped
from pycram.external_interfaces.nav2_move import start_nav_to_pose
from pycram_suturo_demos.helper_methods_and_useful_classes.nlp_human_robot_interaction import TalkingNode
from pycram.motion_executor import ExecutionEnvironment, simulated_robot, real_robot
# from pycram_suturo_demos.helper_methods_and_useful_classes.A_robot_setup import (
#     robot_setup,
# )



# vom lab: door_pos = PoseStamped.from_list(position=[1.3, 1.12, 0], orientation=[0, 0, -0.8197653913600055, 0.5726994876271299])

door_pos = PoseStamped.from_list(position=[1.334370493888855, -1.2590627670288086, 0], orientation=[0, 0, 0.854238273810402, 0.5198816899616919])



# def initialization(simulation: bool = True):
#     logger = logging.getLogger(__name__)
#
#     result = robot_setup(simulation=simulation)
#     rclpy_node, world, robot_view, context = (
#         result.node,
#         result.world,
#         result.robot_view,
#         result.context,
#     )
#     return rclpy_node, world, robot_view, context



def main():
    talk = TalkingNode()

    SIMULATED = False
    robot_type: ExecutionEnvironment = simulated_robot if SIMULATED else real_robot

    #rclpy_node, world, robot_view, context = initialization(simulation=SIMULATED)

    with robot_type:
        talk.pub("I will drive to the door now.", delay=6)
        start_nav_to_pose(nav_pose=door_pos)
        talk.pub("Please open the door for me.", delay=6)



if __name__ == "__main__":
    main()
