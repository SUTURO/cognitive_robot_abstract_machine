from typing import List, Type, TypeVar
from dataclasses import field

from semantic_digital_twin.world import World
from semantic_digital_twin.semantic_annotations.mixins import HasDestination
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Fridge,
    Cupboard,
    Sink,
    GarbageBin,
    Milk,
    Cup,
    Bottle,
    Table,
    HasSupportingSurface,
)
from semantic_digital_twin.semantic_annotations.mixins import IsPerceivable
from semantic_digital_twin.world_description.world_entity import SemanticAnnotation


def query_object_destination(world: World, obj: HasDestination) -> List[SemanticAnnotation]:
    """
    Query suitable destination semantic annotations for a given object.

    The object's class defines one or multiple preferred destination types via
    the `destination_class_names` class variable.

    :param world: The world containing semantic annotations.
    :param obj: The object to be brought somewhere (must support HasDestination).
    :return: A list of all destination semantic annotations found in the world.
             The list may be empty.
    """
    # Try instance attribute first
    dest_types = getattr(obj, "destination_class_names", None)

    # Fall back to class attribute if instance has empty or None
    if not dest_types:
        dest_types = getattr(type(obj), "destination_class_names", None)

    if not dest_types:
        return []

    results: List[SemanticAnnotation] = []
    for dest_type in dest_types:
        results.extend(world.get_semantic_annotations_by_type(dest_type))
    return results