"""
The places a person can sit on a sofa, as regions above its seat.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Human,
    Sofa,
)
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    RegionSpecification,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Scale
from semantic_digital_twin.world_description.world_entity import Region

# %% seat places


class SofaSeat(StrEnum):
    """
    The places a person can sit on a three-seat sofa, named from the left to the right
    of someone sitting on it.

    Each value is the name of the region above that place's seat.
    """

    LEFT = "sofa_seat_left"
    MIDDLE = "sofa_seat_middle"
    RIGHT = "sofa_seat_right"

    def region_in(self, world: World) -> Region:
        """
        :param world: A world whose sofa has seat regions spawned by
            :class:`SofaSeatRegions`.
        :return: The region above this seat place.
        """
        return world.get_kinematic_structure_entity_by_name(self.value)

    def is_free_in(self, world: World) -> bool:
        """
        :param world: A world whose sofa has seat regions spawned by
            :class:`SofaSeatRegions`.
        :return: Whether no :class:`Human` in ``world`` sits on this seat place.
        """
        return not self.region_in(world).semantic_annotations_within(Human)


# %% seat regions


@dataclass
class SofaSeatRegions:
    """
    The regions above the seat of a sofa that faces along its +x axis, one per
    :class:`SofaSeat`, side by side across its width.
    """

    cushion_height: float
    """
    Height of the top of the seat above the sofa's frame, in meters.
    """

    backrest_front: float
    """
    Where along the sofa's x axis the seat begins behind, in meters.
    """

    region_height: float = 0.9
    """
    How far above the seat a seat region reaches, in meters, enough to hold a seated
    adult up to the head.
    """

    def spawn(self, world: World, sofa: Sofa) -> None:
        """
        Spawn the seat regions as children of ``sofa``'s root, reaching from
        :attr:`backrest_front` to the sofa's front edge.

        :param world: The world holding ``sofa``.
        :param sofa: The sofa to seat people on.
        """
        sofa_box = sofa.root.collision.as_bounding_box_collection_in_frame(
            sofa.root
        ).bounding_box()
        seat_width = (sofa_box.max_y - sofa_box.min_y) / len(SofaSeat)
        for seat_index, seat in enumerate(SofaSeat):
            RegionSpecification.box(
                seat.value,
                Scale(
                    sofa_box.max_x - self.backrest_front, seat_width, self.region_height
                ),
                parent_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=(sofa_box.max_x + self.backrest_front) / 2,
                    y=sofa_box.max_y - (seat_index + 0.5) * seat_width,
                    z=self.cushion_height + self.region_height / 2,
                ),
            ).spawn(world, parent=sofa.root)
