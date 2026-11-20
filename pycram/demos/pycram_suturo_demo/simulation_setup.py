import os
import logging
from copy import deepcopy
from dataclasses import dataclass
from typing import Callable, Optional, Sequence, Tuple

from pycram.datastructures.dataclasses import Context
from semantic_digital_twin.adapters.mesh import STLParser
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.semantic_annotations import Milk
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world_description.connections import OmniDrive
from semantic_digital_twin.world_description.world_entity import Body

logger = logging.getLogger(__name__)


def _here(*parts: str) -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), *parts))


@dataclass(frozen=True)
class WorldSetupPaths:
    hsrb_urdf: str
    milk_stl: str
    cereal_stl: str


@dataclass(frozen=True)
class SpawnSpec:
    world_path: str
    xyz_rpy: Tuple[float, float, float, float, float, float]


@dataclass(frozen=True)
class SetupResult:
    world: object
    robot_view: HSRB
    context: Context
    viz: Optional[object]


def default_paths() -> WorldSetupPaths:
    return WorldSetupPaths(
        hsrb_urdf=_here("..", "..", "resources", "robots", "hsrb.urdf"),
        milk_stl=_here("..", "..", "resources", "objects", "milk.stl"),
        cereal_stl=_here("..", "..", "resources", "objects", "breakfast_cereal.stl"),
    )


def build_hsrb_world(hsrb_urdf: str):
    world = URDFParser.from_file(file_path=hsrb_urdf).parse()
    with world.modify_world():
        odom = Body(name=PrefixedName("odom_combined"))
        world.add_kinematic_structure_entity(odom)
        world.add_connection(
            OmniDrive.create_with_dofs(parent=odom, child=world.root, world=world)
        )
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
        world.add_semantic_annotation(Milk(body=world.get_body_by_name("milk.stl")))

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
):
    merged = deepcopy(environment_world)
    x, y, z, r, p, yaw = robot_xyz_rpy
    merged.merge_world_at_pose(
        deepcopy(hsrb_world),
        HomogeneousTransformationMatrix.from_xyz_rpy(x, y, z, r, p, yaw),
    )
    robot_view = HSRB.from_world(merged)
    return merged, robot_view, Context(merged, robot_view)


def try_make_viz(world):
    try:
        import rclpy
        from semantic_digital_twin.adapters.ros.visualization.viz_marker import VizMarkerPublisher

        node = rclpy.create_node("viz_marker")
        return VizMarkerPublisher(world, node)
    except Exception:
        logger.info(
            "VizMarkerPublisher is unavailable (ROS not running or deps missing)."
        )
        return None


def setup_hsrb_in_environment(
    load_environment: Callable[[], object],
    paths: Optional[WorldSetupPaths] = None,
    milk_xyz_rpy: Tuple[float, float, float, float, float, float] = (
        2.37,
        2.0,
        1.05,
        0.0,
        0.0,
        0.0,
    ),
    cereal_xyz_rpy: Tuple[float, float, float, float, float, float] = (
        2.37,
        1.8,
        1.05,
        0.0,
        0.0,
        0.0,
    ),
    robot_xyz_rpy: Tuple[float, float, float, float, float, float] = (
        1.5,
        2.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ),
    with_viz: bool = True,
) -> SetupResult:
    p = paths or default_paths()

    hsrb_world = build_hsrb_world(p.hsrb_urdf)

    env_world = load_environment()
    env_world = add_objects_and_semantics(
        env_world,
        objects=(
            SpawnSpec(world_path=p.milk_stl, xyz_rpy=milk_xyz_rpy),
            SpawnSpec(world_path=p.cereal_stl, xyz_rpy=cereal_xyz_rpy),
        ),
    )

    world, robot_view, context = merge_robot_into_environment(
        hsrb_world, env_world, robot_xyz_rpy=robot_xyz_rpy
    )

    viz = try_make_viz(world) if with_viz else None
    return SetupResult(world=world, robot_view=robot_view, context=context, viz=viz)
