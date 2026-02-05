import os

import numpy as np

from semantic_digital_twin.adapters.mesh import STLParser
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body
from pycram.external_interfaces import robokudo
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from suturo_resources.suturo_map import load_environment


def add_box(name: str, scale_xyz: tuple[float, float, float]):
    body = Body(
        name=PrefixedName(name),
        collision=ShapeCollection([Box(scale=Scale(*scale_xyz))]),
    )
    return body


def perceive_and_spawn_all_objects(hsrb_world):
    perceived_objects_result = robokudo.query_all_objects().res
    perceived_objects = {}
    for perceived_object in perceived_objects_result:
        object_size = perceived_object.shape_size[0].dimensions
        object_pose = perceived_object.pose[0].pose
        object_name = "milk"
        object_to_spawn = add_box(
            object_name, (object_size.x, object_size.y, object_size.z)
        )
        env_world = load_environment()
        perceived_objects[object_name] = object_to_spawn

        with hsrb_world.modify_world():
            hsrb_world.merge_world(env_world)
            hsrb_world.add_connection(
                FixedConnection(
                    parent=hsrb_world.root,
                    child=object_to_spawn,
                    parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_quaternion(
                        pos_x=object_pose.position.x,
                        pos_y=object_pose.position.y,
                        pos_z=object_pose.position.z,
                        quat_x=1.0,
                        quat_y=6.22,
                        quat_z=0.8,
                        quat_w=np.pi / 2,
                    ),
                )
            )
    return perceived_objects
