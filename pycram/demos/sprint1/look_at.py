import threading
import time
import os
import logging

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from suturo_resources.suturo_map import load_environment
from tmc_control_msgs.action import GripperApplyEffort

from pycram.datastructures.enums import (
    TorsoState,
    Arms,
    ApproachDirection,
    VerticalAlignment,
    GripperState,
)
from pycram.datastructures.grasp import GraspDescription
from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan, CodeNode, CodePlan
from pycram.process_module import simulated_robot, real_robot
from semantic_digital_twin.adapters.pose_publisher import PosePublisher
from semantic_digital_twin.adapters.ros.world_fetcher import (
    FetchWorldServer,
    fetch_world_from_service,
)
from semantic_digital_twin.adapters.ros.world_synchronizer import (
    Synchronizer,
    ModelSynchronizer,
    StateSynchronizer,
)
from semantic_digital_twin.adapters.viz_marker import VizMarkerPublisher
from pycram.robot_plans import (
    SimplePouringActionDescription,
    MoveTorsoActionDescription,
    PickUpAction,
    PickUpActionDescription,
    MoveGripperMotion,
    SetGripperActionDescription,
    LookAtActionDescription,
)
from pycram.robot_plans import ParkArmsActionDescription
from pycram.datastructures.dataclasses import Context
from semantic_digital_twin.adapters.mesh import STLParser
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.abstract_robot import ParallelGripper
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
import pycram.alternative_motion_mappings.hsrb_motion_mapping
from pycram.external_interfaces import tmc, robokudo
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

logger = logging.getLogger(__name__)
rclpy.init()


def _here(*parts: str) -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), *parts))


node = rclpy.create_node("viz_marker")
executor = SingleThreadedExecutor()
executor.add_node(node)

thread = threading.Thread(target=executor.spin, daemon=True, name="rclpy-executor")
thread.start()
time.sleep(0.1)

# world from giskard
hsrb_world = fetch_world_from_service(node)
model_sync = ModelSynchronizer(world=hsrb_world, node=node)
state_sync = StateSynchronizer(world=hsrb_world, node=node)


#
# try:
#     hsrb_world.get_body_by_name("bowl.stl")
#     logger.debug("was in true")
# except Exception as e:
#     logger.debug(e)
#     env_world = load_environment()
#     bowl_world = STLParser(
#         os.path.join(
#             os.path.dirname(__file__), "..", "..", "resources", "objects", "bowl.stl"
#         )
#     ).parse()
#     milk_world = STLParser(
#         os.path.join(
#             os.path.dirname(__file__), "..", "..", "resources", "objects", "milk.stl"
#         )
#     ).parse()
#     with hsrb_world.modify_world():
#         hsrb_world.merge_world(env_world)
#         hsrb_world.merge_world_at_pose(
#             milk_world,
#             pose=HomogeneousTransformationMatrix.from_xyz_rpy(
#                 x=1.0, y=6.22, z=0.8, yaw=np.pi / 2
#             ),
#         )
def add_box(name: str, scale_xyz: tuple[float, float, float]):
    body = Body(
        name=PrefixedName(name),
        collision=ShapeCollection([Box(scale=Scale(*scale_xyz))]),
    )
    return body


def add_milk(name: str, scale_xyz: tuple[float, float, float]):
    body = STLParser(
        os.path.join(
            os.path.dirname(__file__), "..", "..", "resources", "objects", "milk.stl"
        )
    ).parse()
    return body


perceived_objects = {}


# Perceive
def perceive_and_spawn_all_objects():
    perceived_objects_result = robokudo.query_all_objects().res
    for perceived_object in perceived_objects_result:
        object_size = perceived_object.shape_size[0].dimensions
        object_pose = perceived_object.pose[0].pose
        object_time = perceived_object.pose[0].header.stamp
        object_name = f"{perceived_object.type}"
        # object_to_spawn = add_box(
        #     object_name,
        #     (object_size.x, object_size.y, object_size.z),
        # )
        object_to_spawn = add_milk(
            object_name,
            (object_size.x, object_size.y, object_size.z),
        )
        env_world = load_environment()
        perceived_objects[object_name] = object_to_spawn
        with hsrb_world.modify_world():
            hsrb_world.merge_world(env_world)
            hsrb_world.merge_world_at_pose(
                object_to_spawn,
                pose=HomogeneousTransformationMatrix.from_xyz_quaternion(
                    pos_x=object_pose.position.x,
                    pos_y=object_pose.position.y,
                    pos_z=object_pose.position.z,
                    quat_x=object_pose.orientation.x,
                    quat_y=object_pose.orientation.y,
                    quat_z=object_pose.orientation.z,
                    quat_w=object_pose.orientation.w,
                ),
            )


VizMarkerPublisher(hsrb_world, node, throttle_state_updates=5)
# PosePublisher(hsrb_world, node)
context = Context(
    hsrb_world, hsrb_world.get_semantic_annotations_by_type(HSRB)[0], ros_node=node
)
gripper = hsrb_world.get_semantic_annotations_by_type(ParallelGripper)[0]
grasp = GraspDescription(ApproachDirection.FRONT, VerticalAlignment.NoAlignment, False)


plan1 = SequentialPlan(
    context,
    LookAtActionDescription(
        target=PoseStamped.from_spatial_type(
            HomogeneousTransformationMatrix.from_xyz_rpy(x=1.0, y=6.22, z=0.8)
        )
    ),
)

with real_robot:
    plan1.perform()
    perceive_and_spawn_all_objects()
