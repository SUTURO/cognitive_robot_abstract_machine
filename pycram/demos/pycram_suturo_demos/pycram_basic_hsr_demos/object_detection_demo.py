import os

import rclpy

from pycram.datastructures.pose import PoseStamped
from pycram.external_interfaces import nav2_move, robokudo
import logging

from pycram.datastructures.enums import Arms
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot
from pycram.robot_plans import ParkArmsActionDescription
from pycram_suturo_demos.helper_methods_and_useful_classes.object_creation import (
    add_milk,
)
from pycram_suturo_demos.pycram_basic_hsr_demos.start_up import setup_hsrb_context
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

logger = logging.getLogger(__name__)
rclpy_node, world, robot_view, context = setup_hsrb_context()


def add_box(name: str, scale_xyz: tuple[float, float, float]):
    body = Body(
        name=PrefixedName(name),
        collision=ShapeCollection([Box(scale=Scale(*scale_xyz))]),
    )
    new_world = World()
    with new_world.modify_world():
        new_world.add_body(body)
    return new_world


def perceive_and_spawn_all_objects():
    perceived_objects = {}
    perceived_objects_result = robokudo.query_all_objects().res
    for perceived_object in perceived_objects_result:
        object_size = perceived_object.shape_size[0].dimensions
        object_pose = perceived_object.pose[0].pose
        object_time = perceived_object.pose[0].header.stamp
        object_name = f"{perceived_object.type}"
        object_to_spawn = add_box(
            object_name,
            (object_size.x, object_size.y, object_size.z),
        )
        # object_to_spawn = add_milk(
        #     object_name,
        #     (object_size.x, object_size.y, object_size.z),
        # )
        perceived_objects[object_name] = object_to_spawn
        with world.modify_world():
            world.merge_world_at_pose(
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
    return perceived_objects


perceive_and_spawn_all_objects()
