import os

import semantic_digital_twin
from pycram.datastructures.dataclasses import Context
from pycram.motion_executor import simulated_robot
from pycram.datastructures.pose import PoseStamped
from pycram.external_interfaces import nav2_move
import logging

from pycram.datastructures.enums import Arms
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot
from pycram.robot_plans import ParkArmsActionDescription, NavigateActionDescription
from demos.pycram_suturo_demos.pycram_basic_hsr_demos.start_up import setup_hsrb_context
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world import World

logging.getLogger(semantic_digital_twin.world.__name__).setLevel(logging.WARN)


def move_demo(simulated: bool, target_pose: str, world: World, context: Context):

    CABINET = PoseStamped.from_list(
        position=[3.8683114051818848, 5.459158897399902, 0.0],
        orientation=[0.0, 0.0, 0.04904329912700753, 0.9987966533838301],
        frame=world.root,
    )
    POPCORN_TABLE = PoseStamped.from_list(
        position=[1.3, 5.3, 0.0],
        orientation=[0.0, 0.0, 0.72, 0.64],
        frame=world.root,
    )
    ROBOT_PRE_START_POSE = PoseStamped.from_list(
        position=[1.3, 0.0, 0.0],
        orientation=[0.0, 0.0, -0.72, 0.64],
        frame=world.root,
    )

    ROBOT_START_POSE = PoseStamped.from_list(
        position=[0.0, 0.0, 0.0],
        orientation=[0, 0, 0, 1],
        frame=world.root,
    )

    def robot_move(target_pose_method: PoseStamped, frame_id: str = "map"):
        """
        Sends a navigation goal to Nav2.
        """
        if simulated:
            with simulated_robot:
                SequentialPlan(
                    context,
                    NavigateActionDescription(
                        target_location=target_pose_method, keep_joint_states=True
                    ),
                ).perform()
        else:
            from pycram.external_interfaces import nav2_move

            os.environ["ROS_PYTHON_CHECK_FIELDS"] = "1"
            goal = target_pose.ros_message()
            print(f"Moving to {goal}'")
            nav2_move.start_nav_to_pose(goal)

    match target_pose:
        case "ROBOT_START_POSE":
            robot_move(target_pose_method=ROBOT_PRE_START_POSE)
            robot_move(target_pose_method=ROBOT_START_POSE)
        case "CABINET":
            robot_move(target_pose_method=CABINET)
        case "POPCORN_TABLE":
            robot_move(target_pose_method=POPCORN_TABLE)
