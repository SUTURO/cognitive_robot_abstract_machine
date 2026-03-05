from typing import Any

import numpy
import rclpy
from rclpy.logging import get_logger

import semantic_digital_twin.exceptions

from pycram.datastructures.enums import Arms
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot, simulated_robot, ExecutionEnvironment
from pycram.robot_plans import (
    ParkArmsActionDescription,
    GiskardPickUpActionDescription,
)
from demos.pycram_suturo_demos.helper_methods_and_useful_classes.robot_setup import (
    robot_setup,
)
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.abstract_robot import ParallelGripper
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Box, Scale, Cylinder
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body
from suturo_resources.suturo_map import load_environment


def try_get_object_to_pickup(world, object_name_method) -> Body | None:
    try:
        object_to_pickup_method = world.get_body_by_name(object_name_method)
        logger.info(f"picking up object with name '{object_name_method}'")

        return object_to_pickup_method
    except semantic_digital_twin.exceptions.WorldEntityNotFoundError:
        raise Exception(f"object with name '{object_name_method}' not found")


def add_cylinder(name: str, width: float, height: float):
    body = Body(
        name=PrefixedName(name),
        collision=ShapeCollection([Cylinder(width=width, height=height)]),
    )
    new_world = World()
    with new_world.modify_world():
        new_world.add_body(body)
    return new_world


def add_box(name: str, scale_xyz: tuple[float, float, float]):
    body = Body(
        name=PrefixedName(name),
        collision=ShapeCollection([Box(scale=Scale(*scale_xyz))]),
    )
    new_world = World()
    with new_world.modify_world():
        new_world.add_body(body)
    return new_world


def perceive_and_spawn_all_objects(hsrb_world: World):
    try:
        from pycram.external_interfaces import robokudo
    except ImportError:
        raise ImportError()
    perceived_objects = {}
    perceived_objects_result = robokudo.query_all_objects().res
    for perceived_object in perceived_objects_result:
        object_size = perceived_object.shape_size[0].dimensions
        object_pose = perceived_object.pose[0].pose
        object_time = perceived_object.pose[0].header.stamp
        object_name = f"{perceived_object.type}"
        try:
            object_to_spawn = hsrb_world.get_body_by_name(object_name)
            with hsrb_world.modify_world():
                hsrb_world.move_branch_to_new_world(object_to_spawn)
        except semantic_digital_twin.exceptions.WorldEntityNotFoundError:
            pass
        object_to_spawn = add_box(
            object_name,
            (object_size.x, object_size.y, object_size.z),
        )
        env_world = load_environment()
        perceived_objects[object_name] = object_to_spawn
        with hsrb_world.modify_world():
            hsrb_world.merge_world(env_world)
            hsrb_world.merge_world_at_pose(
                object_to_spawn,
                pose=HomogeneousTransformationMatrix.from_xyz_quaternion(
                    pos_x=object_pose.position.x,
                    pos_y=object_pose.position.y,
                    pos_z=object_pose.position.z,
                    quat_x=object_pose.orientation.x,
                    quat_y=object_pose.orientation.y,
                    quat_z=object_pose.orientation.z,
                    quat_w=object_pose.orientation.w,
                ),
            )
    return perceived_objects


def attach_object(manipulator: ParallelGripper, world: World, object_designator: Body):
    # Attach the object to the end effector
    with world.modify_world():
        world.move_branch_with_fixed_connection(
            object_designator, manipulator.tool_frame
        )


# ------------------------ BASE-DEFINITIONS
logger = get_logger(__name__)

SIMULATED: bool = True
USE_TEST_CYLINDER: bool = True  # set True to spawn a test cylinder instead of milk.stl
with_perception: bool = False
object_name: str = ""

robot_type: ExecutionEnvironment = simulated_robot if SIMULATED else real_robot


result = robot_setup(SIMULATED)

hsrb_world, robot_view, context, node = (
    result.world,
    result.robot_view,
    result.context,
    result.node,
)

# IMPORTANT: giskardpy's ROS2 ActionClient needs a valid rclpy.node.Node.
# PyCRAM passes context.ros_node down into the MotionExecutor.
if getattr(context, "ros_node", None) is None:
    context.ros_node = node
# Some setups name it differently; set it too if present.
if hasattr(context, "node") and getattr(context, "node", None) is None:
    context.node = node

manipulator = hsrb_world.get_semantic_annotations_by_type(ParallelGripper)[0]
# -------------------------------- DETERMIN OBJECT_TO_PICKUP
if SIMULATED:
    if USE_TEST_CYLINDER:
        object_name = "test_cylinder"
        cylinder_world = add_cylinder(object_name, width=0.06, height=0.2)
        with hsrb_world.modify_world():
            hsrb_world.merge_world_at_pose(
                cylinder_world,
                pose=HomogeneousTransformationMatrix.from_xyz_rpy(x=2.4027, y=-1.5056, z=0.64681, yaw=numpy.pi / 4),
            )
    else:
        object_name = "milk.stl"
    object_to_pickup = try_get_object_to_pickup(hsrb_world, object_name)
else:
    if with_perception:
        from demos.pycram_suturo_demos.helper_methods_and_useful_classes.object_creation import (
            perceive_and_spawn_all_objects,
        )

        perceived_objects: dict[Any, Any] = perceive_and_spawn_all_objects(hsrb_world)
        logger.info(f"perceived following objects: '{perceived_objects}'")
    object_to_pickup = try_get_object_to_pickup(hsrb_world, object_name)
    logger.info(f"object_to_Pickup: '{object_to_pickup}'")

# -------------------------------- PLANNING
plan = SequentialPlan(
    context,
    ParkArmsActionDescription(Arms.BOTH),
    GiskardPickUpActionDescription(object_designator=object_to_pickup, arm=Arms.LEFT),
)

plan2 = SequentialPlan(context, ParkArmsActionDescription(Arms.BOTH))
# ------------------------ EXECUTION
try:
    with simulated_robot:
        plan.perform()
        attach_object(
            world=hsrb_world,
            object_designator=object_to_pickup,
            manipulator=manipulator,
        )
        plan2.perform()
finally:
    # Always shut down cleanly even if planning/execution raises.
    if rclpy.ok():
        rclpy.shutdown()
