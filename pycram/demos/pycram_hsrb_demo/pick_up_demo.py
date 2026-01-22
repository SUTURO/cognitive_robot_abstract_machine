import logging
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from suturo_resources.suturo_map import load_environment

from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import (
    Arms,
    TorsoState,
    ApproachDirection,
    VerticalAlignment,
)
from pycram.datastructures.grasp import GraspDescription
from pycram.datastructures.pose import PoseStamped
from pycram.external_interfaces import robokudo
from pycram.language import SequentialPlan
from pycram.process_module import real_robot
from pycram.robot_plans import (
    LookAtActionDescription,
    ParkArmsActionDescription,
    MoveTorsoActionDescription,
    PickUpActionDescription,
)
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import (
    ModelSynchronizer,
    StateSynchronizer,
)
from semantic_digital_twin.adapters.viz_marker import VizMarkerPublisher
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from pycram.alternative_motion_mappings import hsrb_motion_mapping
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

logger = logging.getLogger(__name__)

rclpy.init()
node = rclpy.create_node("viz_maker")
executor = SingleThreadedExecutor()
executor.add_node(node)
print(node)
# Start executor in a separate thread
thread = threading.Thread(target=executor.spin, daemon=True, name="rclpy-executor")
thread.start()
time.sleep(1)

hsrb_world = fetch_world_from_service(node)
print(hsrb_world.bodies)
model_sync = ModelSynchronizer(world=hsrb_world, node=node)
print(model_sync)
state_sync = StateSynchronizer(world=hsrb_world, node=node)


# Setup context
context = Context(
    hsrb_world, hsrb_world.get_semantic_annotations_by_type(HSRB)[0], ros_node=node
)

grasp = GraspDescription(ApproachDirection.FRONT, VerticalAlignment.NoAlignment, False)


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
        object_name = f"milk"
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
                    parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                        x=object_pose.position.x,
                        y=object_pose.position.y,
                        z=object_pose.position.z,
                        # quat_x=1.0,
                        # quat_y=6.22,
                        # quat_z=0.8,
                        yaw=np.pi / 2,
                    ),
                )
            )
    return perceived_objects


VizMarkerPublisher(hsrb_world, node, throttle_state_updates=5)

perceived_objects = perceive_and_spawn_all_objects(hsrb_world)
print(perceived_objects)


# plan1 = SequentialPlan(
#     context,
#     # LookAtActionDescription(
#     #     target=PoseStamped.from_spatial_type(
#     #         HomogeneousTransformationMatrix.from_xyz_rpy(x=1.0, y=6.22, z=0.8)
#     #     )
#     # ),
#     PickUpActionDescription(
#         arm=Arms.LEFT,
#         object_designator=hsrb_world.get_body_by_name("milk"),
#         grasp_description=grasp,
#     )
# )
#
# plan2 = SequentialPlan(
#     context,
#     ParkArmsActionDescription(arm=Arms.LEFT),
#     MoveTorsoActionDescription(TorsoState.HIGH),
# )

# # Execute the plans
with real_robot:
    SequentialPlan(
        context,
        PickUpActionDescription(
            arm=Arms.LEFT,
            object_designator=hsrb_world.get_body_by_name("milk"),
            grasp_description=grasp,
        ),
    ).perform()

    # plan1.perform()
    # Uncomment to execute plan2
    # plan2.perform()

# exit(0)
# {'milk': Body(name=PrefixedName(name='milk', prefix=None), id=UUID('7e80e94d-980d-42c3-ac43-f25cc04c897d'), index=89, collision_config=CollisionCheckingConfig(buffer_zone_distance=None, violated_distance=0.0, disabled=None, max_avoided_bodies=1), temp_collision_config=None, inertial=Inertial(mass=1.0, center_of_mass=Point3(casadi_sx=SX(@1=0, [@1, @1, @1, 1]), reference_frame=None), inertia=InertiaTensor(data=array([[1., 0., 0.],
#         [0., 1., 0.],
#         [0., 0., 1.]]))))}
