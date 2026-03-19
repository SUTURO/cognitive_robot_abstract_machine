import json
import os

from pycram_suturo_demos.helper_methods_and_useful_classes.semantic_helper_methods import (
    get_object_class_from_string,
)
from semantic_digital_twin.adapters.mesh import STLParser
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.exceptions import WorldEntityNotFoundError
from semantic_digital_twin.semantic_annotations.mixins import HasRootBody
from semantic_digital_twin.spatial_types import (
    Point3,
    Quaternion,
    Vector3,
    HomogeneousTransformationMatrix,
)
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import (
    Body,
    SemanticAnnotation,
)


def add_box(name: str, scale_xyz: tuple[float, float, float]):
    body = Body(
        name=PrefixedName(name),
        collision=ShapeCollection([Box(scale=Scale(*scale_xyz))]),
    )
    return body


def add_milk(name: str, scale_xyz: tuple[float, float, float]):
    body = STLParser(
        os.path.join(
            os.path.dirname(__file__), "..", "..", "resources", "objects", "milk.stl"
        )
    ).parse()
    return body


def extract_name_from_json_string(json_string: str) -> str:
    """
    Extracts the name of an object from a JSON string.
    Expects the JSON string to have a "type" field containing the name of the object.

    :param json_string: The JSON string containing the object information
    :return: The extracted name
    """

    data = json.loads(json_string)
    return data["type"]


def try_remove_semantic_annotation_and_body(name: str, world: World):
    """
    Tries to remove a semantic annotation and its associated body from the world based on the provided name.
    If no annotation with the provided name exists, it does nothing.
    """

    try:
        object_to_remove = world.get_semantic_annotation_by_name(name)
        with world.modify_world():
            world.remove_semantic_annotation(object_to_remove)
            for body in object_to_remove.bodies:
                world.remove_kinematic_structure_entity(body)
    except WorldEntityNotFoundError:
        pass


def _resolve_reference_frame(frame, world: World):
    """
    Resolve a pose reference frame into a KinematicStructureEntity.

    The semantic digital twin APIs expect reference_frame to be a world entity,
    but perception messages often provide ROS frame ids as strings.
    """
    if frame is None:
        return None

    if hasattr(frame, "id"):
        return frame

    if isinstance(frame, str):
        normalized = frame.strip()

        root_names = {
            "map",
            "world",
            str(world.root.name),
        }
        if normalized in root_names:
            return world.root

        try:
            return world.get_body_by_name(normalized)
        except WorldEntityNotFoundError as exc:
            raise ValueError(
                f"Unknown reference frame '{normalized}'. "
                "Expected a world frame/body known to the semantic digital twin."
            ) from exc

    raise TypeError(
        f"Unsupported reference_frame type: {type(frame).__name__}. "
        "Expected None, a frame object, or a frame name string."
    )


def move_object_to_new_pose(
    semantic_annotation: HasRootBody, new_transform: HomogeneousTransformationMatrix
):
    world = semantic_annotation._world
    new_transform_world = world.transform(new_transform, world.root)
    parent_connection = semantic_annotation.root.parent_connection
    parent_connection_parent = parent_connection.parent
    parent_connection_child = parent_connection.child
    new_transform_world.reference_frame = parent_connection_parent
    new_transform_world.child_frame = parent_connection_child
    new_parent_connection = FixedConnection(
        parent=parent_connection_parent,
        child=parent_connection_child,
        parent_T_connection_expression=new_transform_world,
    )
    world.remove_connection(parent_connection)
    world.add_connection(new_parent_connection)


def spawn_semantic_with_body(
    semantic_type: HasRootBody | str,
    name: str,
    scale: Scale,
    pose: Pose,
    world: World,
):
    """
    Spawns a semantic annotation with a body in the world based on the provided information.
    If an annotation with the same name already exists, it is removed before spawning the new one.

    :param semantic_type: The type of the semantic annotation to spawn
    :param name: The name of the semantic annotation to spawn
    :param scale: The scale of the object to spawn
    :param pose: The pose of the object to spawn
    :param world: The world in which to spawn the object
    :return: The spawned semantic annotation
    """

    if isinstance(semantic_type, str):
        semantic_type: HasRootBody = get_object_class_from_string(semantic_type)

    pose.z -= 0.015  # To avoid spawning objects in the air due to small inaccuracies in the pose estimation.

    resolved_reference_frame = _resolve_reference_frame(pose.reference_frame, world)
    pose.reference_frame = resolved_reference_frame

    # If the pose has a frame_id, we need to transform it to the world root frame.
    # Otherwise, we can assume it is already in the world root frame.
    if pose.reference_frame is not None and pose.reference_frame != world.root:
        world_root_T_self = world.transform(pose, world.root).to_homogeneous_matrix()
        print(world_root_T_self)
    else:
        world_root_T_self = pose.to_homogeneous_matrix()
        world_root_T_self.reference_frame = world.root

    try_remove_semantic_annotation_and_body(name, world)

    with world.modify_world():
        object_to_spawn = semantic_type.create_with_new_body_in_world(
            name=PrefixedName(name),
            world=world,
            scale=scale,
            world_root_T_self=world_root_T_self,
        )
    print(object_to_spawn)
    return object_to_spawn


def perceive_and_spawn_all_objects(world: World):
    """
    Query all perceived objects via the robokudo interface, extracts the relevant information for each object,
    and spawns them in the world using the spawn_semantic_with_body method.

    :param world: The world in which to spawn the perceived objects
    """

    try:
        from pycram.external_interfaces import robokudo
    except ImportError:
        raise ImportError()

    perceived_objects_result = robokudo.query_all_objects().res
    print(perceived_objects_result)
    for perceived_object in perceived_objects_result:

        object_dimensions = perceived_object.shape_size[0].dimensions
        object_scale = Scale(
            object_dimensions.x, object_dimensions.y, object_dimensions.z
        )

        object_pose_stamped = perceived_object.pose[0]
        object_pose = Pose(
            position=Point3(
                object_pose_stamped.pose.position.x,
                object_pose_stamped.pose.position.y,
                object_pose_stamped.pose.position.z,
            ),
            orientation=Quaternion(
                object_pose_stamped.pose.orientation.x,
                object_pose_stamped.pose.orientation.y,
                object_pose_stamped.pose.orientation.z,
                object_pose_stamped.pose.orientation.w,
            ),
            reference_frame=object_pose_stamped.header.frame_id,
        )

        # TODO: Needs testing if this is correct and more error handling
        object_name = "muesli_vitalis_box_nutmix8"
        object_type = "cereal"

        spawn_semantic_with_body(
            semantic_type=object_type,
            name=object_name,
            scale=object_scale,
            pose=object_pose,
            world=world,
        )
