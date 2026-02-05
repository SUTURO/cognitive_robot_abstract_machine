import logging
import os
from copy import deepcopy

from suturo_resources.suturo_map import load_environment

from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import TorsoState, Arms
from pycram.language import SequentialPlan
from pycram.process_module import simulated_robot
from pycram.robot_plans import ParkArmsActionDescription, MoveTorsoActionDescription
from semantic_digital_twin.adapters.mesh import STLParser
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.semantic_annotations import Milk
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world_description.connections import OmniDrive
from semantic_digital_twin.world_description.world_entity import Body

logger = logging.getLogger("semantic_digital_twin")
logger.setLevel(logging.DEBUG)


def _here(*parts: str) -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), *parts))


def hsr_world_setup():
    hsr_urdf = _here("..", "..", "resources", "robots", "hsrb.urdf")
    world = URDFParser.from_file(file_path=hsr_urdf).parse()

    with world.modify_world():
        hsr_root = world.root
        odom = Body(name=PrefixedName("odom_combined"))
        world.add_kinematic_structure_entity(odom)
        world.add_connection(
            OmniDrive.create_with_dofs(parent=odom, child=hsr_root, world=world)
        )

    return world


def environment_world_setup():
    world = load_environment()
    milk_stl = _here("..", "..", "resources", "objects", "milk.stl")
    cereal_stl = _here("..", "..", "resources", "objects", "breakfast_cereal.stl")

    milk_world = STLParser(milk_stl).parse()
    cereal_world = STLParser(cereal_stl).parse()

    world.merge_world_at_pose(
        milk_world,
        HomogeneousTransformationMatrix.from_xyz_rpy(
            2.37, 2.0, 1.05, reference_frame=world.root
        ),
    )
    world.merge_world_at_pose(
        cereal_world,
        HomogeneousTransformationMatrix.from_xyz_rpy(
            2.37, 1.8, 1.05, reference_frame=world.root
        ),
    )

    with world.modify_world():
        world.add_semantic_annotation(Milk(body=world.get_body_by_name("milk.stl")))

    return world


def hsr_apartment_world(hsr_world, apartment_world):
    merged = deepcopy(apartment_world)
    merged.merge_world_at_pose(
        deepcopy(hsr_world),
        HomogeneousTransformationMatrix.from_xyz_rpy(1.5, 2.0, 0.0),
    )

    robot_view = HSRB.from_world(merged)
    return merged, robot_view, Context(merged, robot_view)


hsrb_world = hsr_world_setup()
environemnt_world = environment_world_setup()
world, robot_view, context = hsr_apartment_world(hsrb_world, environemnt_world)

try:
    import rclpy

    from semantic_digital_twin.adapters.viz_marker import VizMarkerPublisher

    v = VizMarkerPublisher(world, rclpy.create_node("viz_marker"))
except ImportError:
    logger.info(
        "Could not import VizMarkerPublisher. This is probably because you are not running ROS."
    )


plan = SequentialPlan(
    context,
    ParkArmsActionDescription(Arms.BOTH),
    MoveTorsoActionDescription(TorsoState.HIGH),
)

with simulated_robot:
    plan.perform()
