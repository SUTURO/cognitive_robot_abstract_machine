from typing import List, Type

from semantic_digital_twin.world import World
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    SemanticAnnotation,
    HasDestination,
)

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
    results: List[SemanticAnnotation] = []
    for dest_type in obj.destination_class_names:
        results.extend(world.get_semantic_annotations_by_type(dest_type))
    return results
