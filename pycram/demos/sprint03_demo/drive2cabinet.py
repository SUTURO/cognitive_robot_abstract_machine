import logging
from enum import Enum

from pycram.datastructures.dataclasses import Context
from pycram.datastructures.pose import PoseStamped
from pycram.external_interfaces import nav2_move
from pycram.ros import VizMarkerPublisher
from pycram.ros_utils.text_to_image import TextToImagePublisher
from .pycram_ros_setup import setup_ros_node
from semantic_digital_twin.robots.hsrb import HSRB

logger = logging.getLogger(__name__)


class DriveToPosition(Enum):
    CABINET = PoseStamped.from_list(
        position=[3.8683114051818848, 5.459158897399902, 0.0],
        orientation=[0.0, 0.0, 0.04904329912700753, 0.9987966533838301],
    )


# Setup ROS node and fetch the world
node, hsrb_world = setup_ros_node()

# Setup context
context = Context(
    hsrb_world, hsrb_world.get_semantic_annotations_by_type(HSRB)[0], ros_node=node
)

VizMarkerPublisher()

# Drive to positions
text_pub = TextToImagePublisher()
for pos in DriveToPosition:
    text_pub.publish_text(f"Moving to: {pos.name}")
    goal = pos.value.ros_message()
    goal.header.frame_id = "map"
    nav2_move.start_nav_to_pose(goal)
