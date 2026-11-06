"""
The places a person can sit on a sofa, as regions above its seat.
"""

from __future__ import annotations

from enum import StrEnum

from semantic_digital_twin.semantic_annotations.semantic_annotations import Human
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Region

# %% seat places


class SofaSeat(StrEnum):
    """
    The places a person can sit on a two-seat sofa, named from the left to the right of
    someone sitting on it.

    Each value is the name of the region above that place's seat cushion.
    """

    LEFT = "sofa_seat_left"
    RIGHT = "sofa_seat_right"

    def region_in(self, world: World) -> Region:
        """
        :param world: A world whose sofa has a region for every seat place.
        :return: The region above this seat place.
        """
        return world.get_kinematic_structure_entity_by_name(self.value)

    def is_free_in(self, world: World) -> bool:
        """
        :param world: A world whose sofa has a region for every seat place.
        :return: Whether no :class:`Human` in ``world`` sits on this seat place.
        """
        return not self.region_in(world).semantic_annotations_within(Human)
