import threading
import time
from dataclasses import dataclass

import rclpy
from rclpy.executors import SingleThreadedExecutor
import logging
from suturo_resources.suturo_map import load_environment
from typing_extensions import Tuple, Any

import semantic_digital_twin.exceptions
from pycram.datastructures import dataclasses
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot
from pycram.robot_plans import ParkArmsActionDescription
from semantic_digital_twin.adapters.ros.messages import LoadModel
from semantic_digital_twin.adapters.ros.tf_publisher import TFPublisher
from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
    VizMarkerPublisher,
)
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import (
    ModelSynchronizer,
    StateSynchronizer,
    ModelReloadSynchronizer,
)
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.abstract_robot import AbstractRobot
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection
import numpy as np

from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body
from test.krrood_test.dataset.example_classes import Node

logger = logging.getLogger(__name__)


import threading
import rclpy
from rclpy.executors import SingleThreadedExecutor


def setup_hsrb_context(
    node_name: str = "pycram_node",
) -> Tuple[Any, World, HSRB, Context]:
    """
    Initializes rclpy, starts a SingleThreadedExecutor in a background thread,
    synchronizes the world model, and returns all relevant objects.

    Returns:
        dict containing:
            - node
            - world
            - robot_view
            - context
    """

    # Initialize ROS 2
    rclpy.init()

    # Create node
    rclpy_node = rclpy.create_node(node_name)

    # Create executor
    executor = SingleThreadedExecutor()
    executor.add_node(rclpy_node)

    # Start executor in background thread
    thread = threading.Thread(
        target=executor.spin,
        daemon=True,
        name="rclpy-executor",
    )
    thread.start()

    # Fetch world
    world: World = fetch_world_from_service(rclpy_node)

    # Synchronizers
    model_sync = ModelSynchronizer(world=world, node=rclpy_node)
    state_sync = StateSynchronizer(world=world, node=rclpy_node)

    # Optional TF publisher
    # TFPublisher(world=world, node=rclpy_node)

    env_world = load_environment()
    with world.modify_world():
        world.merge_world(env_world)

    # Visualization
    VizMarkerPublisher(world=world, node=rclpy_node)

    # Robot semantic view
    robot_view = world.get_semantic_annotations_by_type(HSRB)[0]

    # Context
    context = Context(
        world,
        robot_view,
        ros_node=rclpy_node,
    )

    return rclpy_node, world, robot_view, context
