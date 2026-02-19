from enum import Enum

from suturo_resources.suturo_map import load_environment
import logging

from demos.helper_methods_and_useful_classes.setup_real_robot import setup_ros_node
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.pose import PoseStamped
from pycram.external_interfaces import nav2_move
from semantic_digital_twin.robots.hsrb import HSRB

logger = logging.getLogger(__name__)


class Positions(Enum):
    CABINET = PoseStamped.from_list(
        position=[3.8683114051818848, 5.459158897399902, 0.0],
        orientation=[0.0, 0.0, 0.04904329912700753, 0.9987966533838301],
    )


def real_demo():
    # Should be run as plan. But I don't think it is implemented yet
    node, hsrb_world = setup_ros_node()

    env_world = load_environment()
    hsrb_world.merge_world(env_world)

    try:
        hsrb_world.get_body_by_name("environment")
    except Exception as e:
        logger.debug(e)
        env_world = load_environment()
        with hsrb_world.modify_world():
            hsrb_world.merge_world(env_world)

    context = Context(
        hsrb_world,
        hsrb_world.get_semantic_annotations_by_type(HSRB)[0],
        ros_node=node,
    )

    # plan = SequentialPlan(
    #     context, ParkArmsActionDescription(arm=Arms.BOTH), NavigateActionDescription()
    # )

    # So directly call the interface for now
    for pos in Positions:
        print("Moving to pose: ", pos.name)
        goal = pos.value.ros_message()
        goal.header.frame_id = "map"
        nav2_move.start_nav_to_pose(goal)


if __name__ == "__main__":
    real_demo()
