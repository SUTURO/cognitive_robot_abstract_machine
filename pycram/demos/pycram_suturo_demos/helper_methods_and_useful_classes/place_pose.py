import logging
from typing import List, overload

from semantic_digital_twin.semantic_annotations.mixins import (
    HasRootBody,
    HasSupportingSurface,
)
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import SemanticAnnotation

logger = logging.getLogger(__name__)


def get_poses_on_semantic_annotation_for_object(
    semantic_annotation: SemanticAnnotation | str,
    for_object: SemanticAnnotation | str,
    world: World,
    amount_of_locations: int = 100,
) -> List[Pose] | None:
    """
    Get poses on a semantic annotation for a given object.
    The method samples points from the surface of the semantic annotation and returns them as a list of poses.

    :param semantic_annotation: The name of the semantic annotation to sample points from.
    :param for_object: The object for which the poses are to be sampled. Can be given as a string (name of the object) or directly as the SemanticAnnotation.
    :param world: The world in which the semantic annotation and the object exist.
    :param amount_of_locations: The amount of poses to sample from the surface of the semantic annotation. Default is 100.
    :return: A list of poses on the semantic annotation for the given object,
    or None if no semantic annotation with the given name is found
    or if the semantic annotation has no supporting surface.
    """

    # Get semantic annotation if given as string and check if it has supporting surface
    if isinstance(semantic_annotation, str):
        semantic_annotation: SemanticAnnotation = world.get_semantic_annotation_by_name(
            for_object
        )
    if not isinstance(semantic_annotation, HasSupportingSurface):
        logger.warning(
            f'Semantic annotation with name "{semantic_annotation}" has no supporting surface. Cannot sample points from surface.'
        )
        return None

    # Get for_object if given as string and check if it of Type HasRootBody
    if isinstance(for_object, str):
        for_object: SemanticAnnotation = world.get_semantic_annotation_by_name(
            for_object
        )
    if not isinstance(for_object, HasRootBody):
        logger.warning(
            f'Object with name "{for_object.name}" has no root body. Cannot sample points for object.'
        )
        return None

    points = semantic_annotation.sample_points_from_surface(
        for_object, amount=amount_of_locations
    )

    poses = []
    for point in points:
        poses.append(Pose(position=point, reference_frame=point.reference_frame))
    return poses if len(poses) > 0 else None


def get_pose_on_semantic_annotation_for_object_by_semantic_annotation(
    semantic_annotation: SemanticAnnotation | str,
    for_object: SemanticAnnotation | str,
    world: World,
) -> Pose | None:
    """
    Get a single pose on a semantic annotation for a given object.
    This method is a wrapper around get_poses_on_semantic_annotation_for_object that returns only the first pose from the list of poses.

    :param semantic_annotation: The name of the semantic annotation to sample points from.
    :param for_object: The object for which the poses are to be sampled. Can be given as a string (name of the object) or directly as the SemanticAnnotation.
    :param world: The world in which the semantic annotation and the object exist.
    :return: A list of poses on the semantic annotation for the given object,
    or None if no semantic annotation with the given name is found
    or if the semantic annotation has no supporting surface.
    """
    poses = get_poses_on_semantic_annotation_for_object(
        for_object=for_object,
        semantic_annotation=semantic_annotation,
        world=world,
        amount_of_locations=1,
    )
    return poses[0] if poses is not None else None
