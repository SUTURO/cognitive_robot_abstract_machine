"""
The kitchen lab, with the regions above the seat places of its sofa.
"""

from semantic_digital_twin.predefined_maps.kitchen_environment import (
    KitchenEnvironment,
)

from robokudo.world_descriptor import BaseWorldDescriptor


class WorldDescriptor(BaseWorldDescriptor):
    """
    The kitchen lab of :class:`KitchenEnvironment`, so its regions are the ones the
    plans asking the queries know.
    """

    def __init__(self) -> None:
        super().__init__()
        KitchenEnvironment().populate(self.world)
