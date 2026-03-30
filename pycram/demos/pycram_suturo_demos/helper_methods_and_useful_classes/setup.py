import os
import threading
from typing import Tuple

import rclpy
from rclpy.executors import SingleThreadedExecutor
from suturo_resources.suturo_map import load_environment

from pycram.datastructures.dataclasses import Context
from pycram.motion_executor import simulated_robot, ExecutionEnvironment
from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
    VizMarkerPublisher,
)
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import (
    ModelSynchronizer,
    StateSynchronizer,
)
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import OmniDrive
from semantic_digital_twin.world_description.world_entity import Body


def _real_setup(node_name: str) -> Context:
    """
    Initializes rclpy, starts a SingleThreadedExecutor in a background thread,
    synchronizes the world model, and returns the context.

    Returns: context
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
    print(world.root)

    # Synchronizers
    model_sync = ModelSynchronizer(_world=world, node=rclpy_node, synchronous=True)
    state_sync = StateSynchronizer(_world=world, node=rclpy_node, synchronous=True)

    env_world = load_environment()
    with world.modify_world():
        world.merge_world(env_world)

    # Robot semantic view
    robot_view = world.get_semantic_annotations_by_type(HSRB)[0]

    # Context
    context = Context(
        world=world,
        robot=robot_view,
        ros_node=rclpy_node,
    )

    return context


def _build_hsrb_world():
    hsrb_urdf = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "../../../resources/robots/hsrb.urdf")
    )
    world = URDFParser.from_file(file_path=hsrb_urdf).parse()
    with world.modify_world():
        odom = Body(name=PrefixedName("odom_combined"))
        world.add_kinematic_structure_entity(odom)
        omni_drive = OmniDrive.create_with_dofs(
            parent=odom, child=world.root, world=world
        )
        world.add_connection(omni_drive)
        omni_drive.has_hardware_interface = True
    return world


def _simulated_setup(
    *,
    node_name: str,
    robot_pose: Pose,
) -> Context:
    """
    Initializes rclpy, creates a world with the HSR robot at the specified pose,
    and returns the context.

    Returns: context
    """

    rclpy.init()
    rclpy_node = rclpy.create_node(node_name)

    hsrb_world = _build_hsrb_world()
    env_world = load_environment()

    env_world.merge_world_at_pose(
        hsrb_world,
        robot_pose.to_homogeneous_matrix(),
    )
    robot_view = HSRB.from_world(env_world)

    viz = VizMarkerPublisher(_world=env_world, node=rclpy_node).with_tf_publisher()

    return Context(
        world=env_world,
        robot=robot_view,
        ros_node=rclpy_node,
    )


def setup_context(
    *,
    simulated: bool = True,
    node_name: str = "pycram_node",
    robot_pose: Pose = Pose.from_xyz_quaternion(pos_x=0.5, pos_y=0.2),
) -> Tuple[Context, ExecutionEnvironment]:
    """
    Sets up the context for either a simulated or real robot.
    Returns the context and the execution type.
    """

    if simulated:
        context = _simulated_setup(node_name=node_name, robot_pose=robot_pose)
        execution_type = simulated_robot
    else:
        context = _real_setup(node_name=node_name)
        execution_type = simulated_robot

    return context, execution_type
