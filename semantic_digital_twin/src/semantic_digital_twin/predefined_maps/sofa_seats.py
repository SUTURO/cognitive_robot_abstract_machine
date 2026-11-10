"""
The places a person can sit on a sofa, as regions above its seat.
"""

from __future__ import annotations

from enum import StrEnum

from semantic_digital_twin.semantic_annotations.semantic_annotations import Human
from semantic_digital_twin.specifications.semantic_annotations import (
    SemanticAnnotationWithRootSpecification,
)
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

    @classmethod
    def free_in(cls, world: World) -> list[SofaSeat]:
        """
        :param world: A world whose sofa has a region for every seat place.
        :return: The seat places no :class:`Human` in ``world`` sits on, from left to
            right.
        """
        return [
            seat
            for seat in cls
            if not seat.region_in(world).semantic_annotations_within(Human)
        ]

    def region_in(self, world: World) -> Region:
        """
        :param world: A world whose sofa has a region for every seat place.
        :return: The region above this seat place.
        """
        return world.get_kinematic_structure_entity_by_name(self.value)

    def place_person_in(
        self, world: World, person: SemanticAnnotationWithRootSpecification[Human]
    ) -> Human:
        """
        Sit ``person`` on this seat place, its root fixed to the sofa at the origin of
        this seat place's region.

        :param world: A world whose sofa has a region for every seat place.
        :param person: The person to sit down.
        :return: The person sitting on this seat place.
        """
        region = self.region_in(world)
        sofa_body = region.parent_kinematic_structure_entity
        return person.spawn(
            world,
            parent=sofa_body,
            parent_T_self=world.compute_forward_kinematics(sofa_body, region),
        )
