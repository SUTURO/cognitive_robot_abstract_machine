from object_creation import perceive_and_spawn_all_objects
from pycram.datastructures.enums import TorsoState
from pycram.process_module import real_robot
from pycram.robot_plans import (
    PlaceActionDescription,
)
from pycram_ros_setup import setup_ros_node

import logging

from suturo_resources.queries import eql_is_supported_by2
from suturo_resources.suturo_map import load_environment

from pycram.datastructures.enums import (
    TorsoState,
    Arms,
    GripperState,
)
from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.process_module import simulated_robot, real_robot

# from semantic_digital_twin.adapters.viz_marker import VizMarkerPublisher
from pycram.robot_plans import (
    SimplePouringActionDescription,
    MoveTorsoActionDescription,
)
from pycram.robot_plans import ParkArmsActionDescription
from pycram.datastructures.dataclasses import Context
from semantic_digital_twin.adapters.viz_marker import VizMarkerPublisher
from semantic_digital_twin.robots.hsrb import HSRB
from pycram.external_interfaces import tmc

logger = logging.getLogger(__name__)

# Setup ROS node and fetch the world
node, hsrb_world = setup_ros_node()

env_world = load_environment()
hsrb_world.merge_world(env_world)

# Setup context
context = Context(
    hsrb_world,
    hsrb_world.get_semantic_annotations_by_type(HSRB)[0],
    ros_node=node,
)

# Perceive objects and spawn them
# perceived_objects = perceive_and_spawn_all_objects(hsrb_world)
# print(perceived_objects)

# ---- Adding end-effector and current object in hand

# ----

table_height = (
    (hsrb_world.get_body_by_name("cookingTable_body").global_pose.z) * 2
) + 0.05
print(f"table height: {table_height}")

debug = hsrb_world.get_bodies_by_name("lowerTable_body")
table_pose = hsrb_world.get_bodies_by_name("cookingTable_body")

# print(env_world.get_semantic_annotations_by_name("cookingTable_body"))
# print(env_world.get_body_by_name("cookingTable_body").global_pose.x)
# print(env_world.get_body_by_name("cookingTable_body").global_pose.y)

# ---- PoseStamped erzeugen
print(f"table pose: {table_pose}")
# ----

"""
PoseStamped.from_spatial_type(
            HomogeneousTransformationMatrix.from_xyz_rpy(x=1.0, y=6.22, z=0.8)
        )
"""
VizMarkerPublisher(hsrb_world, node, throttle_state_updates=5)


plan1 = SequentialPlan(
    context,
    PlaceActionDescription(
        table, arm=Arms.LEFT, target=PoseStamped.from_spatial_type(table_pose)
    ),
)

plan2 = SequentialPlan(
    context,
    ParkArmsActionDescription(arm=Arms.LEFT),
    MoveTorsoActionDescription(TorsoState.HIGH),
)

# Execute the plans
# with real_robot:
# plan2.perform()
# Uncomment to execute plan2
# plan2.perform()

exit(0)
