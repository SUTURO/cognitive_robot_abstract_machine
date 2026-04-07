import logging
import os
import threading
import time

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
)
from semantic_digital_twin.spatial_types import (
    HomogeneousTransformationMatrix,
)
from semantic_digital_twin.world_description.connections import (
    FixedConnection,
    OmniDrive,
)
from suturo_resources.suturo_map_rody import load_environment
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


from suturo_resources.suturo_map_rody import load_environment

from test.conftest import hsr_world_setup
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.adapters.ros import world_fetcher, world_synchronizer

try:
    import rclpy

    rclpy.init()
    from semantic_digital_twin.adapters.viz_marker import VizMarkerPublisher

    node = rclpy.create_node("test")
    exec = SingleThreadedExecutor()
    exec.add_node(node)
    thread = threading.Thread(target=exec.spin, daemon=True, name="rclpy-executor")
    thread.start()
    time.sleep(0.1)
except ImportError:
    pass

sem_logger = logging.getLogger("semantic_digital_twin")
sem_logger.setLevel(logging.DEBUG)

initworld = world_fetcher.fetch_world_from_service(node)
print("here-1")

world_synchronizer.StateSynchronizer(initworld, node)
world_synchronizer.ModelSynchronizer(initworld, node)
world = load_environment()
print("here1")

# test objects
cup = STLParser(
    os.path.join(
        os.path.dirname(__file__), "..", "..", "resources", "objects", "jeroen_cup.stl"
    )
).parse()
#
#
#
# urdf_dir = os.path.join(
#         os.path.dirname(os.path.abspath(__file__)),
#         "..",
#         "..",
#         "resources",
#         "robots",
#     )
# hsr = os.path.join(urdf_dir, "hsrb.urdf")
# hsr_parser = URDFParser.from_file(file_path=hsr)
# world_with_hsr = hsr_parser.parse()
# with world_with_hsr.modify_world():
#     hsr_root = world_with_hsr.root
#     localization_body = Body(name=PrefixedName("odom_combined"))
#     world_with_hsr.add_kinematic_structure_entity(localization_body)
#     c_root_bf = OmniDrive.create_with_dofs(
#         parent=localization_body, child=hsr_root, world=world_with_hsr
#     )
#     world_with_hsr.add_connection(c_root_bf)

with initworld.modify_world():
    initworld.merge_world_at_pose(
        cup,
        HomogeneousTransformationMatrix.from_xyz_quaternion(
            2.4, 2.2, 1, reference_frame=initworld.root
        ),
    )
    initworld.merge_world(world)


# while True:
#     print("test")


hsrb = HSRB.from_world(initworld)
context = Context.from_world(initworld)
context.ros_node = node

with initworld.modify_world():
    world_reasoner = WorldReasoner(initworld)
    world_reasoner.reason()
    initworld.add_semantic_annotations(
        [
            Cup(body=initworld.get_body_by_name("jeroen_cup.stl")),
        ]
    )
print("here")
plan = SequentialPlan(
    context,
    ParkArmsActionDescription(Arms.BOTH),
    MoveTorsoActionDescription(TorsoState.HIGH),
)

with real_robot:
    plan.perform()
