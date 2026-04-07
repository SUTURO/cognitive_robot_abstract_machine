import math
import time
from dataclasses import dataclass
from typing import Any

import rclpy
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.motion_executor import simulated_robot
from pycram.ros.ros2.ros_tools import wait_for_message
from pycram_suturo_demos.helper_methods_and_useful_classes.A_robot_setup import (
    robot_setup,
)
from pycram.datastructures.enums import PickUpType
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.abstract_robot import ParallelGripper
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix, Point3
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Box, Scale, Color
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

from suturo_resources.suturo_map import load_environment

import semantic_digital_twin
import logging

logger = logging.getLogger(__name__)


def initialization(simulation: bool, with_simulated_objects: bool = False):
    """
    Initializes the robot setup and returns the core components needed for demo execution.

    :param simulation: Whether to run in simulation mode.
    :param with_simulated_objects: Whether to populate the world with simulated objects.
    :return: Tuple of (node, world, robot_view, context, manipulator).
    """
    result = robot_setup(
        simulation=simulation,
        with_simulated_objects=with_simulated_objects,
    )
    logger.info("initialization done")
    return (
        result.node,
        result.world,
        result.robot_view,
        result.context,
        result.manipulator,
    )


def try_get_object_to_pickup(world: World, object_name_method: str) -> Body | None:
    """
    Tries to retrieve a body from the world by name.
    Raises an exception if the object is not found.

    :param world: The world to search in.
    :param object_name_method: The name of the object to retrieve.
    :return: The Body with the given name.
    :raises Exception: If no object with the given name exists in the world.
    """
    try:
        object_to_pickup_method = world.get_body_by_name(object_name_method)
        logger.info(f"picking up object with name '{object_name_method}'")
        return object_to_pickup_method
    except semantic_digital_twin.exceptions.WorldEntityNotFoundError:
        raise Exception(f"object with name '{object_name_method}' not found")


def add_box(name: str, scale_xyz: tuple[float, float, float]):
    """
    Creates a new world containing a single box-shaped body with the given name and scale.

    :param name: The name to assign to the body.
    :param scale_xyz: A tuple of (x, y, z) dimensions for the box.
    :return: The newly created world containing the box body.
    """
    body = Body(
        name=PrefixedName(name),
        collision=ShapeCollection([Box(scale=Scale(*scale_xyz))]),
    )
    new_world = World()
    with new_world.modify_world():
        new_world.add_body(body)
    return new_world


def perceive_and_spawn_all_objects(world: World) -> dict:
    """
    Queries all objects via RoboKudo, spawns them into the world at their perceived poses,
    and returns a dict mapping object names to their worlds.
    Raises ImportError if RoboKudo is not available.

    :param world: The world in which to spawn the perceived objects.
    :return: A dict mapping object name strings to their spawned worlds.
    :raises ImportError: If the RoboKudo interface cannot be imported.
    """
    try:
        from pycram.external_interfaces import robokudo
    except ImportError:
        raise ImportError()
        return {}
    perceived_objects = {}
    perceived_objects_result = robokudo.query_all_objects().res
    for perceived_object in perceived_objects_result:
        object_size = perceived_object.shape_size[0].dimensions
        object_pose = perceived_object.pose[0].pose
        object_time = perceived_object.pose[0].header.stamp
        object_name = f"{perceived_object.type}"
        try:
            object_to_spawn = world.get_body_by_name(object_name)
            with world.modify_world():
                world.move_branch_to_new_world(object_to_spawn)
        except semantic_digital_twin.exceptions.WorldEntityNotFoundError:
            pass
        object_to_spawn = add_box(
            object_name,
            (object_size.x, object_size.y, object_size.z),
        )
        env_world = load_environment()
        perceived_objects[object_name] = object_to_spawn
        with world.modify_world():
            world.merge_world(env_world)
            world.merge_world_at_pose(
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


def attach_object_to_hsrb(world: World, object_designator: Body):
    """
    Attaches the given object to the HSR-B's end effector tool frame.
    This is a workaround since attaching within Actions does not work with motions.

    :param world: The world in which to attach the object.
    :param object_designator: The body to attach to the gripper.
    """
    manipulator = world.get_semantic_annotations_by_type(ParallelGripper)[0]
    with world.modify_world():
        world.move_branch_with_fixed_connection(
            object_designator, manipulator.tool_frame
        )


def detach_object_from_hsrb(world: World, object_designator: Body):
    """
    Detaches the given object from the HSR-B by re-parenting it to the world root.
    This is a workaround since detaching within Actions does not work with motions.

    :param world: The world in which to detach the object.
    :param object_designator: The body to detach from the gripper.
    """
    with world.modify_world():
        world.move_branch_with_fixed_connection(object_designator, world.root)


def try_perceiving_and_spawning_and_find_object(
    world: World, object_name: str
) -> Body | None:
    """
    Attempts to perceive and spawn all objects via RoboKudo, then retrieves
    the object matching the given name. Gracefully handles missing RoboKudo import.

    :param world: The world in which to spawn perceived objects.
    :param object_name: The name of the object to find after spawning.
    :return: The Body matching the given name, or None if not found.
    """
    try:
        from demos.pycram_suturo_demos.helper_methods_and_useful_classes.object_creation import (
            perceive_and_spawn_all_objects,
        )

        perceived_objects: dict[Any, Any] = perceive_and_spawn_all_objects(world)
        logger.info(f"perceived following objects: '{perceived_objects}'")
    except ImportError:
        logger.info("Could not import robokudo")
        perceived_objects = {}
    object_to_pickup = try_get_object_to_pickup(world, object_name)
    logger.info(f"object_to_Pickup: '{object_to_pickup}'")
    return object_to_pickup


def get_nearest_object(world: World) -> Body | None:
    """
    Returns the nearest object to the robot on the cooking table,
    based on Euclidean distance in the X-Y plane.

    :param world: The world to query.
    :return: The nearest Body on the cooking table, or None if the table is empty.
    """
    robot_view = world.get_semantic_annotations_by_type(HSRB)[0]
    cooking_table_annotation = world.get_semantic_annotation_by_name("cooking_table")
    nearest_objects_list = query_get_next_object_euclidean_x_y(
        robot_view.root, cooking_table_annotation
    ).tolist()
    return nearest_objects_list[0].bodies[0] if nearest_objects_list else None


def get_object_with_color(world: World, color: Color) -> Body | None:
    """
    Returns the first object on the cooking table that matches the given color.

    :param world: The world to query.
    :param color: The color to filter objects by.
    :return: The first matching Body, or None if no matching object is found.
    """
    robot_view = world.get_semantic_annotations_by_type(HSRB)[0]
    cooking_table_annotation = world.get_semantic_annotation_by_name("cooking_table")
    objects_on_table = query_semantic_annotations_on_surfaces(
        [cooking_table_annotation], world
    ).tolist()
    colored_objects = query_annotations_by_color(color, objects_on_table)
    return colored_objects[0].bodies[0] if colored_objects else None


def parse_color(color_str: str) -> Color:
    """
    Converts a color name string to a Color object.
    Falls back to white if the color string is not recognized.

    :param color_str: A color name string (e.g. 'red', 'blue', 'green').
    :return: The corresponding Color object, or Color.WHITE() if unrecognized.
    """
    color_map = {
        "red": Color.RED(),
        "yellow": Color.YELLOW(),
        "green": Color.GREEN(),
        "cyan": Color.CYAN(),
        "blue": Color.BLUE(),
        "magenta": Color.MAGENTA(),
        "white": Color.WHITE(),
        "black": Color.BLACK(),
        "gray": Color.GRAY(),
        "grey": Color.GRAY(),
        "beige": Color.BEIGE(),
        "orange": Color.ORANGE(),
    }
    return color_map.get(color_str.strip().lower(), Color.WHITE())


def get_pickup_mode() -> tuple[PickUpType, str, Color]:
    """
    Prompts the user to select a pickup mode and any required parameters.
    Returns the selected mode along with the object name or color if applicable.

    :return: A tuple of (PickUpType, object_name, object_color).
             object_name is empty string if not applicable.
             object_color defaults to Color.WHITE() if not applicable.
    """
    object_color = Color.WHITE()
    object_name = ""
    mode_map = {
        "nearest object": PickUpType.PICK_UP_OBJECT_BY_NEAREST,
        "object by color": PickUpType.PICK_UP_OBJECT_BY_COLOR,
        "object by name": PickUpType.PICK_UP_OBJECT_SEARCH,
    }

    mode_str = input(
        "Pick up by name, color, or nearest object? (e.g: nearest object) "
    )
    mode = mode_map.get(mode_str.strip().lower(), PickUpType.PICK_UP_OBJECT_BY_NEAREST)

    match mode:
        case PickUpType.PICK_UP_OBJECT_SEARCH:
            object_name = input("Which object do you want to pick up? ")
            logger.info(f"Looking for object: {object_name}")
        case PickUpType.PICK_UP_OBJECT_BY_COLOR:
            color_str = input(
                "Which color should the object be? (e.g: red, blue, green) "
            )
            object_color = parse_color(color_str)
            logger.info(f"Object color: {object_color}")

    return mode, object_name, object_color


def object_to_pickup_by_mode(
    world: World, mode: PickUpType, object_name: str = "", color: Color = Color.WHITE()
) -> Body | None:
    """
    Resolves which object to pick up based on the given pickup mode. This is best performed RIGHT before pickup, since the nearest_object method uses the robots position

    :param world: The world to search for objects in.
    :param mode: The pickup strategy to use.
    :param object_name: The name of the object (used for PICK_UP_OBJECT_SEARCH mode).
    :param color: The color to filter by (used for PICK_UP_OBJECT_BY_COLOR mode).
    :return: The resolved Body to pick up, or None if no suitable object was found.
    """
    match mode:
        case PickUpType.PICK_UP_OBJECT_SEARCH:
            object_to_pickup = try_perceiving_and_spawning_and_find_object(
                world=world, object_name=object_name
            )
            logger.info(f"object_to_pickup by name: {object_to_pickup}")
        case PickUpType.PICK_UP_OBJECT_BY_COLOR:
            object_to_pickup = get_object_with_color(world, color)
            logger.info(f"object_to_pickup by color: {object_to_pickup}")
        case _:
            object_to_pickup = get_nearest_object(world=world)
            logger.info(f"object_to_pickup by nearest: {object_to_pickup}")
    return object_to_pickup


def item_between_fingertips(
    fingertip_distance: float,
    closed_value: float = -0.1007,
    open_value: float = 0.0538,
    threshhold: float = 0.05,
) -> bool:
    """
    Returns True if the gripper is not fully closed and not fully open,
    which can indicate that an item is between the fingertips.

    Args:
        fingertip_distance: Current value from /gripper_command/fingertip_distance
        closed_value: Typical fully closed value
        open_value: Typical fully open value
        threshhold: Tolerance around the reference values

    Returns:
        True if the distance suggests an object is between the fingertips.
    """
    closed_min = closed_value - threshhold
    closed_max = closed_value + threshhold
    open_min = open_value - threshhold
    open_max = open_value + threshhold

    is_closed = closed_min <= fingertip_distance <= closed_max
    is_open = open_min <= fingertip_distance <= open_max

    # Object likely present if it is neither clearly open nor clearly closed
    return not is_closed and not is_open


def validate_grasped() -> bool:
    node = rclpy.create_node("gripper_distance_subscriber")

    msg = wait_for_message(
        msg_type=float, node=node, topic_name="/gripper_command/fingertip_distance"
    )
    success = msg is not None
    if success:
        logger.info(f"Gripper fingertip distance: {msg.data}")
    else:
        logger.warning("Timed out waiting for gripper fingertip distance")
    node.destroy_node()

    is_object_between_fingertips = item_between_fingertips(fingertip_distance=msg)
    return is_object_between_fingertips


@dataclass
class PickupDeadzone:
    min_distance: float = 0.3  # too close, robot can't reach down
    max_distance: float = 0.8  # too far to reach
    max_angle_deg: float = 45.0  # cone in front of robot (±45°)
    max_height_diff: float = 0.2  # object must be near floor level


def is_in_pickup_zone(self, object_position: tuple[float, float, float]) -> bool:
    """
    object_position: (x, y, z) in robot's base frame
    x = forward, y = left, z = up
    """
    ox, oy, oz = object_position

    # Horizontal distance from robot base
    distance = math.sqrt(ox**2 + oy**2)

    # Too close or too far
    if distance < self.deadzone.min_distance:
        logger.debug(f"Object too close: {distance:.2f}m")
        return False
    if distance > self.deadzone.max_distance:
        logger.debug(f"Object too far: {distance:.2f}m")
        return False

    # Outside forward cone
    angle_deg = math.degrees(math.atan2(abs(oy), ox))
    if angle_deg > self.deadzone.max_angle_deg:
        logger.debug(f"Object outside reach cone: {angle_deg:.1f}°")
        return False

    # Object too high or too low
    if abs(oz) > self.deadzone.max_height_diff:
        logger.debug(f"Object height out of range: {oz:.2f}m")
        return False

    return True


def try_percieve_and_retrieve(
    simulated: bool = False,
    context: Context = None,
    angle: int = 1,
    talking_node: Any = None,
    object_name: str = None,
) -> Body | None:
    from pycram_suturo_demos.pycram_basic_hsr_demos.move_demo import move_demo

    world = context.world
    robot_view = context.robot
    standard_delay = 2
    table = world.get_body_by_name("cooking_table")
    look_at = HomogeneousTransformationMatrix.to_position(table.global_pose)

    talking_node.pub(
        text=f"Trying to position, to perceive object.", delay=standard_delay
    )
    move_demo(
        simulated=simulated,
        world=world,
        context=context,
        target_pose="PERCEPTION_ANGLE_" + str(angle),
    )
    look_at_point(context, look_at)
    perceive_and_spawn_all_objects(world)

    try:
        object_to_pickup: Body | None = world.get_body_by_name(object_name)
        talking_node.pub(
            text=f"Found object {object_to_pickup.name}.", delay=standard_delay
        )
        return object_to_pickup
    except Exception:
        object_to_pickup: Body | None = None
        talking_node.pub(
            text=f"Could not find object {object_name}.",
            delay=standard_delay,
        )
        time.sleep(2)
        return object_to_pickup


# Stolen from ansgar
def look_at_point(context: Context, point: Point3):
    from pycram.robot_plans import LookAtActionDescription

    with simulated_robot:
        SequentialPlan(
            context,
            LookAtActionDescription(
                PoseStamped.from_spatial_type(
                    HomogeneousTransformationMatrix.from_point_rotation_matrix(
                        point=point, reference_frame=point.reference_frame
                    )
                )
            ),
        ).perform()
