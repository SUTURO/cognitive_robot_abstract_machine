import logging
import os
import threading
import time
from copy import deepcopy

from rclpy.executors import SingleThreadedExecutor

from pycram.process_module import real_robot
from semantic_digital_twin.adapters.urdf import URDFParser

from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import TorsoState, Arms
from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.process_module import simulated_robot
from pycram.robot_plans import MoveTorsoActionDescription, TransportActionDescription
from pycram.robot_plans import ParkArmsActionDescription
from pycram.testing import setup_world
from semantic_digital_twin.adapters.mesh import STLParser
from semantic_digital_twin.reasoning.world_reasoner import WorldReasoner
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Bowl,
    Spoon,
    Cup,
    Milk,
)
from semantic_digital_twin.spatial_types import (
    HomogeneousTransformationMatrix,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import (
    FixedConnection,
    OmniDrive,
)
from suturo_resources.suturo_map import load_environment
from rclpy.node import Node
from semantic_digital_twin.adapters.mesh import STLParser
from semantic_digital_twin.reasoning.world_reasoner import WorldReasoner
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.semantic_annotations import Container
from semantic_digital_twin.world_description.connections import FixedConnection
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import TorsoState, Arms
from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.process_module import simulated_robot
from pycram.robot_plans import MoveTorsoActionDescription, TransportActionDescription
from pycram.robot_plans import ParkArmsActionDescription
from test.conftest import hsr_world_setup
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.adapters.ros import world_fetcher, world_synchronizer
from importlib import resources
from pathlib import Path


logger = logging.getLogger("semantic_digital_twin")
logger.setLevel(logging.DEBUG)

try:
    from semantic_digital_twin.adapters.viz_marker import VizMarkerPublisher
except ImportError:
    logger.info(
        "Could not import VizMarkerPublisher. This is probably because you are not running ROS."
    )


def hsr_world_setup():
    urdf_dir = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..",
        "pycram",
        "resources",
        "robots",
    )
    hsr = os.path.join(urdf_dir, "hsrb.urdf")
    hsr_parser = URDFParser.from_file(file_path=hsr)
    world_with_hsr = hsr_parser.parse()
    with world_with_hsr.modify_world():
        hsr_root = world_with_hsr.root
        localization_body = Body(name=PrefixedName("odom_combined"))
        world_with_hsr.add_kinematic_structure_entity(localization_body)
        c_root_bf = OmniDrive.create_with_dofs(
            parent=localization_body, child=hsr_root, world=world_with_hsr
        )
        world_with_hsr.add_connection(c_root_bf)

    return world_with_hsr


def setup_world() -> World:
    logger.setLevel(logging.DEBUG)

    hsrb_sem_world = hsr_world_setup
    suturo_world = load_environment()
    milk_world = STLParser(
        os.path.join(
            os.path.dirname(__file__), "..", "..", "resources", "objects", "milk.stl"
        )
    ).parse()
    # apartment_world.merge_world(pr2_sem_world)
    suturo_world.merge_world(milk_world)

    with suturo_world.modify_world():
        suturo_world.get_body_by_name("milk.stl").parent_connection.origin = (
            HomogeneousTransformationMatrix.from_xyz_rpy(
                2.37, 2, 1.05, reference_frame=suturo_world.root
            )
        )
        milk_view = Milk(body=suturo_world.get_body_by_name("milk.stl"))
        with suturo_world.modify_world():
            suturo_world.add_semantic_annotation(milk_view)
        return suturo_world


world = setup_world()
robot_view = HSRB.from_world(world)
context = Context.from_world(world, robot_view)
original_state_data = deepcopy(world.state.data)


# with world.modify_world():
#     world_reasoner = WorldReasoner(world)
#     world_reasoner.reason()
#     world.add_semantic_annotations(
#         [
#             Cup(body=world.get_body_by_name("jeroen_cup.stl")),
#         ]
#     )

plan = SequentialPlan(
    context,
    ParkArmsActionDescription(Arms.BOTH),
    MoveTorsoActionDescription(TorsoState.HIGH),
)

with simulated_robot:
    plan.perform()
