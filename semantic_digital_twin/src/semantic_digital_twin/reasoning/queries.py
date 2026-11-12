from __future__ import annotations

from krrood.inheritance_path_length import (inheritance_distance)

from krrood.entity_query_language.factories import not_, set_of, type_
import math
from krrood.entity_query_language.factories import (
    variable_from,
    entity,
    contains,
    variable,
    an,
)
from krrood.entity_query_language.query.query import Entity
from krrood.entity_query_language.predicate import symbolic_function, length
from krrood.utils import recursive_subclasses

from semantic_digital_twin.reasoning.predicates import (
    SupportedBy,
    Supports,
    PlaceIsOccupied,
    compute_euclidean_planar_distance, OverFloorPlan, GrowingCircleSearch,
)
from semantic_digital_twin.semantic_annotations.mixins import (
    HasSupportingSurface,
    IsPerceivable,
    HasRootBody,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Room,
    Floor,
)
from semantic_digital_twin.world import World
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Pose
from semantic_digital_twin.world_description.geometry import (
    Color,
    VolumetricBoundingBox,
)
from semantic_digital_twin.world_description.world_entity import (
    Body,
    SemanticAnnotation,
)


def semantic_annotations_on_surfaces(
    supporting_surfaces: list[HasSupportingSurface], world: World
) -> list[HasRootBody]:
    """
    Queries a list of Semantic annotations that are on top of a given list of other
    annotations (ex.

    Tables). param: supporting_surfaces: List of SemanticAnnotations that are supporting other annotations.
    :param world: World object that contains the supporting_surfaces.
    return: List of SemanticAnnotations that are supported by the given supporting_surfaces.
    """
    objects = []
    with world.modify_world():
        for surface in supporting_surfaces:
            surface.infer_objects_on_surface()
            objects.extend(surface.objects)

    return objects


def get_next_object_using_planar_distance(
    main_body: Body,
    supporting_surface,
    ignore_dimension,
) -> Entity[SemanticAnnotation]:
    """
    Queries the next object based on Euclidean distance in x and y coordinates relative
    to the given main body and supporting surface. This function utilizes semantic
    annotations of objects and orders them by their Euclidean distances to the main
    body.

    :param main_body: The main body to which the Euclidean distance is computed.
    :param supporting_surface: The surface on which the semantic annotations of interest
        are queried.
    :return: A `QueryObjectDescriptor` containing semantic annotations ordered by
        Euclidean distance to the main body.
    """
    supported_semantic_annotations = variable_from(
        semantic_annotations_on_surfaces([supporting_surface], main_body._world)
    )
    return entity(supported_semantic_annotations).ordered_by(
        compute_euclidean_planar_distance(
            body1=supported_semantic_annotations.bodies[0],
            body2=main_body,
            ignore_dimension=ignore_dimension,
        )
    )


def goal_surface_of_object(
    object_of_interest: SemanticAnnotation,
    supporting_surfaces: list[HasSupportingSurface],
    threshold: int = 1,
) -> HasSupportingSurface | None:
    """
    Finds the most similar object to a given semantic annotation among a list of tables
    based on the inheritance path length. If the similarity does not meet the provided
    threshold, the method attempts to return the table that is not supporting any
    object. The similarity metric leverages the class hierarchy to compute distances.

    :param object_of_interest: The semantic annotation to compare.
    :param supporting_surfaces: A list of supporting surfaces semantic annotations to
        search on top of them for similar objects to the object_of_interest.
    :param threshold: The maximum acceptable inheritance path length to classify objects
        as similar. Defaults to 1.
    :return: The semantic annotation of the most appropriate surface based on similarity
        metrics or the non-supporting table when no viable candidate is found, or None
        if there are no supporting surfaces.
    """
    if not supporting_surfaces:
        return None
    supporting_surface = variable(HasSupportingSurface, supporting_surfaces)
    supporting_body = supporting_surface.bodies[0]
    non_supporting_table = entity(supporting_surface).where(
        not_(Supports(supporting_body))
    )

    # Query annotations on the surfaces of the tables
    obj = variable(
        SemanticAnnotation,
        semantic_annotations_on_surfaces(
            supporting_surfaces, object_of_interest._world
        ),
    )

    query = (
        set_of(obj, supporting_surface)
        .where(
            (distance := inheritance_distance(object_of_interest, type_(obj)))
            <= threshold,
            SupportedBy(obj.bodies[0], supporting_body),
        )
        .ordered_by(distance)
    )
    return next(
        query[supporting_surface].evaluate(),
        next(non_supporting_table.evaluate(), None),
    )


def filter_annotations_by_color(
    color: Color, objects: list[SemanticAnnotation]
) -> Entity[SemanticAnnotation]:
    """
    Queries and retrieves a list of annotations from another one that match the
    specified color based on their visual properties.

    :param color: The color to filter annotations by.
    :param objects: The list of the unfiltered annotations.
    :return: A list of annotations from the world whose primary shape's visual color
        matches the specified color.
    """
    if len(objects) == 0:
        return entity(variable_from(objects))
    world = objects[0]._world
    object_var = variable_from(objects)
    body = object_var.bodies[0]

    matching_body = entity(body).where(
        body.visual != None,
        length(body.visual.shapes) > 0,
        body.visual.shapes[0].color == color,
    )
    semantic_annotation = variable(HasRootBody, world.semantic_annotations)
    return entity(semantic_annotation).where(semantic_annotation.root == matching_body)


def annotation_class_by_label(label: str) -> type | None:
    """
    Finds the class whose name is contained within the given label.

    It searches through all subclasses of IsPerceivable.

    :param label: The string input from perception (e.g.,
        "bowl_collapsable_yellowgrey").
    :return: The matching class (e.g., Bowl) or None if no match is found.
    """

    @symbolic_function
    def class_name_lowercased(semantic_class: type) -> str:
        """
        The class's own name, lowercased, for case-insensitive label matching.
        """
        return semantic_class.__name__.lower()

    semantic_class = variable_from(recursive_subclasses(IsPerceivable))
    matching_class = an(
        entity(semantic_class).where(
            contains(label.lower(), class_name_lowercased(semantic_class))
        )
    )
    return next(matching_class.evaluate(), None)


def sort_annotations_by_volume(
    annotations: list[HasRootBody], order: bool | None = True
) -> list[HasRootBody]:
    """
    Sorts a list of SemanticAnnotations by volume in descending order (largest to
    smallest). Volume is calculated by multiplying the scale dimensions (x * y * z) of
    the object's shape.

    :param annotations: List of annotations of type HasRootBody to sort.
    :param order: Whether to sort in ascending or descending order (default is True).
    :return: List of SemanticAnnotation objects sorted by volume (largest to smallest).
    """
    annotaion_var = entity(a := variable_from(annotations)).where(a.bodies)

    @symbolic_function
    def get_volume(annotation: SemanticAnnotation) -> float:
        """
        Calculate volume from the annotation's body scale.
        """
        body = annotation.bodies[0]

        # Get shapes from collision if available, otherwise from visual
        if body.collision is not None:
            return (
                body.collision.scale.x * body.collision.scale.y * body.collision.scale.z
            )
        else:
            return 0.0

    return entity(annotaion_var).ordered_by(
        get_volume(annotaion_var), descending=not order
    )


def find_room(room: Room, world: World) -> (float, float):
    for sem in world.get_semantic_annotations_by_type(room):
        return (float(sem.bodies[0].global_pose.x), float(sem.bodies[0].global_pose.y))
    return (0, 0)


# %% free space for the robot


def find_free_space_in_room(
    room_type: type[Room],
    world: World,
    robot_diameter: float = 0.5,
    robot_height: float = 1.35,
    radius_step: float = 0.1,
    maximum_radius: float = 3.0,
    preferred_point: Pose | None = None,
) -> Pose | None:
    """
    Finds the place nearest to the pose :func:`find_room` gives for a room where a robot
    fits without touching anything, within that room. Of equally near places, the one
    nearest to the preferred point wins.

    :param room_type: The type of the room to stand in.
    :param world: The world holding the room.
    :param robot_diameter: The diameter of the robot's footprint, in m.
    :param robot_height: The height of the robot, in m.
    :param radius_step: See :attr:`GrowingCircleSearch.radius_step`.
    :param maximum_radius: See :attr:`GrowingCircleSearch.maximum_radius`.
    :param preferred_point: See :attr:`GrowingCircleSearch.preferred_point`.
    :return: The pose of the robot's base on the floor, or None if no place within the
        maximum radius is free.
    """
    [room] = world.get_semantic_annotations_by_type(room_type)
    room_x, room_y = find_room(room_type, world)
    free_center = _nearest_free_robot_center(
        GrowingCircleSearch(
            center=Pose.from_xyz_rpy(
                room_x, room_y, robot_height / 2, reference_frame=world.root
            ),
            radius_step=radius_step,
            maximum_radius=maximum_radius,
            preferred_point=preferred_point,
        ),
        room,
        world,
        robot_diameter,
        robot_height,
    )
    if free_center is None:
        return None
    return Pose.from_xyz_rpy(
        float(free_center.x), float(free_center.y), 0, reference_frame=world.root
    )


def find_free_space_next_to(
    semantic_annotation: HasRootBody,
    robot_diameter: float = 0.5,
    robot_height: float = 1.35,
    radius_step: float = 0.1,
    maximum_radius: float = 3.0,
    preferred_point: Pose | None = None,
) -> Pose | None:
    """
    Finds the place nearest to a semantic annotation where a robot fits without touching
    anything, within the annotation's room, facing the annotation. Of equally near
    places, the one nearest to the preferred point wins.

    :param semantic_annotation: The annotation to drive to, such as a table.
    :param robot_diameter: The diameter of the robot's footprint, in m.
    :param robot_height: The height of the robot, in m.
    :param radius_step: See :attr:`GrowingCircleSearch.radius_step`.
    :param maximum_radius: See :attr:`GrowingCircleSearch.maximum_radius`.
    :param preferred_point: See :attr:`GrowingCircleSearch.preferred_point`.
    :return: The pose of the robot's base on the floor, or None if no place within the
        maximum radius is free.
    """
    world = semantic_annotation._world
    target = semantic_annotation.root.global_pose
    target_x, target_y = float(target.x), float(target.y)
    free_center = _nearest_free_robot_center(
        GrowingCircleSearch(
            center=Pose.from_xyz_rpy(
                target_x, target_y, robot_height / 2, reference_frame=world.root
            ),
            radius_step=radius_step,
            maximum_radius=maximum_radius,
            preferred_point=preferred_point,
        ),
        find_objects_room(semantic_annotation),
        world,
        robot_diameter,
        robot_height,
    )
    if free_center is None:
        return None
    free_x, free_y = float(free_center.x), float(free_center.y)
    return Pose.from_xyz_rpy(
        free_x,
        free_y,
        0,
        yaw=math.atan2(target_y - free_y, target_x - free_x),
        reference_frame=world.root,
    )


def _nearest_free_robot_center(
    search: GrowingCircleSearch,
    room: Room | None,
    world: World,
    robot_diameter: float,
    robot_height: float,
) -> Pose | None:
    """
    :param search: The poses to try, in the order to try them.
    :param room: The room the robot has to stay in, or None to stand anywhere.
    :param world: The world whose bodies the robot must not touch.
    :param robot_diameter: The diameter of the robot's footprint, in m.
    :param robot_height: The height of the robot, in m.
    :return: The first pose of the search at which a robot centred there touches
        nothing but the floor, or None if there is none.
    """
    half_width = robot_diameter / 2
    robot_box = VolumetricBoundingBox(
        -half_width,
        -half_width,
        -robot_height / 2,
        half_width,
        half_width,
        robot_height / 2,
        HomogeneousTransformationMatrix(reference_frame=world.root),
    )
    floor_bodies = [
        body
        for floor in world.get_semantic_annotations_by_type(Floor)
        for body in floor.bodies
    ]
    robot_center = variable(Pose, list(search))
    conditions = [] if room is None else [OverFloorPlan(robot_center, room.floor.root)]
    conditions.append(
        not_(PlaceIsOccupied(robot_box, robot_center, world, floor_bodies))
    )
    query = an(entity(robot_center).where(*conditions))
    return next(query.evaluate(), None)


def find_objects_in_room(
    object_type: type[HasRootBody], room_type: type[Room], world: World
) -> list[HasRootBody]:
    """
    Finds every semantic annotation of a type whose root body stands over the floor plan
    of a room.

    :param object_type: The type of the annotations to find, e.g. :class:`Apple`.
    :param room_type: The type of the room to search in, e.g. :class:`Kitchen`.
    :param world: The world holding the room and the annotations.
    :return: The annotations of the given type inside the room.
    """
    [room] = world.get_semantic_annotations_by_type(room_type)
    annotation = variable(object_type, world.semantic_annotations)
    query = entity(annotation).where(
        OverFloorPlan(annotation.root.global_pose, room.floor.root)
    )
    return list(query.evaluate())


def find_objects_room(semantic_annotation: HasRootBody) -> Room | None:
    """
    Finds the room whose floor plan the root body of a semantic annotation stands over.

    :param semantic_annotation: The annotation to locate.
    :return: The room the annotation is in, or None if it is in no room.
    """
    room = variable(Room, semantic_annotation._world.semantic_annotations)
    query = an(
        entity(room).where(
            OverFloorPlan(semantic_annotation.root.global_pose, room.floor.root)
        )
    )
    return next(query.evaluate(), None)