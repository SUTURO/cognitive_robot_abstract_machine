import threading
from typing import Optional, Any

import time
from dataclasses import dataclass, field

import rclpy
from rclpy.executors import SingleThreadedExecutor
import logging

from semantic_digital_twin.adapters.ros.messages import LoadModel
from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
    VizMarkerPublisher,
)
from semantic_digital_twin.robots.abstract_robot import Manipulator, ParallelGripper
from suturo_resources.suturo_map import load_environment

from pycram.datastructures.dataclasses import Context
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import (
    ModelSynchronizer,
    StateSynchronizer,
    ModelReloadSynchronizer,
)
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection
import numpy as np

from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body


logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class SetupResult:
    world: World
    robot_view: HSRB
    context: Context
    manipulator: Manipulator
    node: Any
    viz: Optional[object] = None


def setup_ros_node(node_name: str = "pycram_node"):
    node = rclpy.create_node(node_name)
    logger.info("Node created, please kill correctly after termination.")

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # Start executor in a separate thread
    thread = threading.Thread(target=executor.spin, daemon=True, name="rclpy-executor")
    thread.start()

    hsrb_world: World = fetch_world_from_service(node)
    model_sync = ModelSynchronizer(world=hsrb_world, node=node)
    state_sync = StateSynchronizer(world=hsrb_world, node=node)

    env_world = load_environment()
    with hsrb_world.modify_world():
        hsrb_world.merge_world(env_world)
    robot_view = hsrb_world.get_semantic_annotations_by_type(HSRB)[0]

    manipulator: Manipulator = hsrb_world.get_semantic_annotations_by_type(
        ParallelGripper
    )[0]

    # Context
    context = Context(
        hsrb_world,
        robot_view,
        ros_node=node,
    )

    return hsrb_world, robot_view, context, manipulator, node


def add_box(name: str, scale_xyz: tuple[float, float, float]):
    body = Body(
        name=PrefixedName(name),
        collision=ShapeCollection([Box(scale=Scale(*scale_xyz))]),
    )
    return body


def test_spawning(hsrb_world: World):
    object_name = f"milk"
    object_to_spawn = add_box(object_name, (0.1, 0.1, 0.3))
    env_world = load_environment()

    with hsrb_world.modify_world():
        hsrb_world.merge_world(env_world)
        hsrb_world.add_connection(
            FixedConnection(
                parent=hsrb_world.root,
                child=object_to_spawn,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=1.51,
                    y=1.9,
                    z=0.5,
                    # quat_x=1.0,
                    # quat_y=6.22,
                    # quat_z=0.8,
                    yaw=np.pi / 2,
                ),
            )
        )


def try_make_viz(world: World, node: Any) -> Optional[VizMarkerPublisher]:
    try:
        import rclpy
        from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
            VizMarkerPublisher,
        )

        viz = VizMarkerPublisher(world=world, node=node)
        viz.with_tf_publisher()

        return viz
    except Exception:
        logger.info(
            "VizMarkerPublisher is unavailable (ROS not running or deps missing)."
        )
        return None


def world_setup_with_test_objects(
    with_object: bool = True,
    with_viz: bool = field(kw_only=True, default=False),
) -> SetupResult:
    rclpy.init()

    hsrb_world, robot_view, context, manipulator, node = setup_ros_node()

    # if with_viz:
    #     viz = try_make_viz(hsrb_world, node)

    return SetupResult(
        world=hsrb_world,
        robot_view=robot_view,
        context=context,
        manipulator=manipulator,
        node=node,
    )
