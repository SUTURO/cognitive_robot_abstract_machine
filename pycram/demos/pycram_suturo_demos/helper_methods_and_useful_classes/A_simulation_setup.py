import os

import rclpy
from rclpy.logging import get_logger
from copy import deepcopy
from dataclasses import dataclass, field
from typing import Callable, Optional, Sequence, Tuple, Any

from pycram.datastructures.dataclasses import Context
from semantic_digital_twin.adapters.mesh import STLParser
from semantic_digital_twin.adapters.ros.visualization.viz_marker import VizMarkerPublisher

from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.abstract_robot import Manipulator, ParallelGripper
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Milk,
    Cereal,
    Bowl,
)
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import OmniDrive
from semantic_digital_twin.world_description.geometry import Color
from semantic_digital_twin.world_description.world_entity import Body

logger = get_logger(__name__)


def _here(*parts: str) -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), *parts))


@dataclass(frozen=True)
class WorldSetupPaths:
    hsrb_urdf: str
    milk_stl: str
    cereal_stl: str
    bowl_stl: str
    cup_stl: str


@dataclass(frozen=True)
class SpawnSpec:
    world_path: str
    xyz_rpy: Tuple[float, float, float, float, float, float]


@dataclass(frozen=True)
class SetupResult:
    world: World
    robot_view: HSRB
    context: Context
    manipulator: Manipulator
    node: Any
    viz: Optional[object] = None


def default_paths() -> WorldSetupPaths:
    return WorldSetupPaths(
        hsrb_urdf=_here("..", "..", "..", "resources", "robots", "hsrb.urdf"),
        milk_stl=_here("..", "..", "..", "resources", "objects", "milk.stl"),
        cereal_stl=_here(
            "..", "..", "..", "resources", "objects", "breakfast_cereal.stl"
        ),
        bowl_stl=_here("..", "..", "..", "resources", "objects", "bowl.stl"),
        cup_stl=_here("..", "..", "..", "resources", "objects", "jeroen_cup.stl"),
    )


def build_hsrb_world(hsrb_urdf: str):
    world = URDFParser.from_file(file_path=hsrb_urdf).parse()
    with world.modify_world():
        odom = Body(name=PrefixedName("odom_combined"))
        world.add_kinematic_structure_entity(odom)
        omni_drive = OmniDrive.create_with_dofs(parent=odom, child=world.root, world=world)
        world.add_connection(omni_drive)
        omni_drive.has_hardware_interface = True
    return world


def add_objects_and_semantics(
    world,
    objects: Sequence[SpawnSpec],
):
    for spec in objects:
        obj_world = STLParser(spec.world_path).parse()
        x, y, z, r, p, yaw = spec.xyz_rpy
        world.merge_world_at_pose(
            obj_world,
            HomogeneousTransformationMatrix.from_xyz_rpy(
                x, y, z, r, p, yaw, reference_frame=world.root
            ),
        )

    with world.modify_world():
        world.add_semantic_annotation(Milk(root=world.get_body_by_name("milk.stl")))
        for c in world.get_body_by_name("milk.stl").visual.shapes:
            c.color = Color.RED()
        world.add_semantic_annotation(
            Cereal(root=world.get_body_by_name("breakfast_cereal.stl"))
        )
        for c in world.get_body_by_name("breakfast_cereal.stl").visual.shapes:
            c.color = Color.BLUE()
        world.add_semantic_annotation(Bowl(root=world.get_body_by_name("bowl.stl")))
        for c in world.get_body_by_name("bowl.stl").visual.shapes:
            c.color = Color.GREEN()
        world.add_semantic_annotation(
            Bowl(root=world.get_body_by_name("jeroen_cup.stl"))
        )
        for c in world.get_body_by_name("jeroen_cup.stl").visual.shapes:
            c.color = Color.ORANGE()
    return world


def merge_robot_into_environment(
    hsrb_world,
    environment_world,
    robot_xyz_rpy: Tuple[float, float, float, float, float, float] = (
        1.5,
        2.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ),
) -> Tuple[World, HSRB, Context, Manipulator]:
    x, y, z, r, p, yaw = robot_xyz_rpy
    environment_world.merge_world_at_pose(
        deepcopy(hsrb_world),
        HomogeneousTransformationMatrix.from_xyz_rpy(x, y, z, r, p, yaw),
    )
    robot_view = HSRB.from_world(environment_world)
    manipulator = environment_world.get_semantic_annotations_by_type(ParallelGripper)[0]
    return (
        environment_world,
        robot_view,
        Context(environment_world, robot_view),
        manipulator,
    )


def try_make_viz(world):
    try:
        import rclpy
        from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
            VizMarkerPublisher,
        )

        node = rclpy.create_node("viz_marker")
        viz = VizMarkerPublisher(_world=world, node=node)
        viz.with_tf_publisher()
        return viz
    except Exception:
        logger.info(
            "VizMarkerPublisher is unavailable (ROS not running or deps missing)."
        )
        return None


def setup_hsrb_in_environment(
    load_environment: Callable[[], World],
    paths: Optional[WorldSetupPaths] = None,
    milk_xyz_rpy: Tuple[float, float, float, float, float, float] = (
        1.2,
        6.6,
        0.78,
        0.0,
        0.0,
        0.0,
    ),
    cereal_xyz_rpy: Tuple[float, float, float, float, float, float] = (
        0.6,
        6.3,
        0.805,
        0.0,
        0.0,
        0.0,
    ),
    bowl_xyz_rpy: Tuple[float, float, float, float, float, float] = (
        1.5,
        6.3,
        0.725,
        0.0,
        0.0,
        0.0,
    ),
    cup_xyz_rpy: Tuple[float, float, float, float, float, float] = (
        1.5,
        6.7,
        0.72,
        0.0,
        0.0,
        0.0,
    ),
    robot_xyz_rpy: Tuple[float, float, float, float, float, float] = (
        0,
        0,
        0.0,
        0.0,
        0.0,
        0.0,
    ),
    with_objects: bool = field(kw_only=True, default=True),
) -> SetupResult:
    rclpy.init()
    p = paths or default_paths()

    node: Any = rclpy.create_node("simulation_setup")

    hsrb_world = build_hsrb_world(p.hsrb_urdf)
    env_world: World = load_environment()

    if with_objects:
        env_world = add_objects_and_semantics(
            env_world,
            objects=(
                SpawnSpec(world_path=p.milk_stl, xyz_rpy=milk_xyz_rpy),
                SpawnSpec(world_path=p.cereal_stl, xyz_rpy=cereal_xyz_rpy),
                SpawnSpec(world_path=p.bowl_stl, xyz_rpy=bowl_xyz_rpy),
                SpawnSpec(world_path=p.cup_stl, xyz_rpy=cup_xyz_rpy),
            ),
        )

    world, robot_view, context, manipulator = merge_robot_into_environment(
        hsrb_world, env_world, robot_xyz_rpy=robot_xyz_rpy
    )

    viz = VizMarkerPublisher(_world=world, node=node).with_tf_publisher()

    return SetupResult(
        world=world,
        robot_view=robot_view,
        context=context,
        node=node,
        manipulator=manipulator,
        viz=viz,
    )
