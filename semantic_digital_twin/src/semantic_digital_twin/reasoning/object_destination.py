from typing import List, Type

from semantic_digital_twin.world import World
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    SemanticAnnotation,
    HasDestination,
)

def query_object_destination(world: World, obj: SemanticAnnotation) -> List[SemanticAnnotation]:
    """
    Query suitable destination semantic annotations for a given object.

    The object can specify one or multiple destination types via the HasDestination mixin.
    Destinations are expressed as semantic annotation class names (strings).

    :param world: The world containing semantic annotations.
    :param obj: The object to be brought somewhere.
    :return: A list of all destination semantic annotations found in the world.
             The list may be empty.
    :raises ValueError: If a destination class name is unknown.
    """
    if not isinstance(obj, HasDestination):
        return []

    destination_names = obj.destination_class_names
    if not destination_names:
        return []

    results: List[SemanticAnnotation] = []
    for class_name in destination_names:
        dest_type = _resolve_semantic_annotation_type(class_name)
        results.extend(world.get_semantic_annotations_by_type(dest_type))

    return results


def _resolve_semantic_annotation_type(class_name: str) -> Type[SemanticAnnotation]:
    """
    Resolve a semantic annotation class name to the actual type.

    :param class_name: Name of a semantic annotation class, e.g. "Fridge".
    :return: The resolved semantic annotation type.
    :raises ValueError: If the class name cannot be resolved.
    """
    from semantic_digital_twin.semantic_annotations import semantic_annotations as sa

    attr = getattr(sa, class_name, None)
    if attr is None or not isinstance(attr, type) or not issubclass(attr, SemanticAnnotation):
        raise ValueError(f"Unknown semantic annotation class name: {class_name}")

    return attr