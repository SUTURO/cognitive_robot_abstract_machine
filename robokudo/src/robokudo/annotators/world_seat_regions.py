"""
The seat places of the shared world as seats in view of the camera.
"""

# %% imports

from __future__ import annotations

import numpy as np
from py_trees.common import Status
from semantic_digital_twin.adapters.ros.node_registry import ROSNodeRegistry
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Chair,
    HasSeatPlaces,
)
from typing_extensions import TYPE_CHECKING

from robokudo.annotators.core import BaseAnnotator
from robokudo.annotators.seat_detection import SeatFurniture, WorldSeatRegion
from robokudo.cas import CASViews
from robokudo.types.annotation import PositionAnnotation
from robokudo.types.cv import Rect
from robokudo.utils.annotator_helper import get_world_to_camera_transform_matrix
from robokudo.utils.o3d_helper import project_points_to_image
from robokudo.utils.region import region_obb_in_camera_coordinates

if TYPE_CHECKING:
    from semantic_digital_twin.world import World

# %% annotator


class WorldSeatRegionAnnotator(BaseAnnotator):
    """
    Adds a :class:`~robokudo.annotators.seat_detection.WorldSeatRegion` to the CAS for
    every seat place of the shared world that lies in front of the camera and within the
    color image.

    Each is positioned at the center of its seat place in the camera frame and its image
    region frames the seat place in the color image.
    """

    def __init__(self, name: str = "WorldSeatRegionAnnotator") -> None:
        """
        :param name: Name of the annotator in the pipeline.
        """
        super().__init__(name)
        self.world: World | None = None
        """
        The shared world with the seat places, fetched on the first update.
        """

    def update(self) -> Status:
        """
        Add the seat regions of the seat places in view to the CAS.

        :return: SUCCESS after processing.
        """
        if self.world is None:
            node = ROSNodeRegistry().get()
            self.world = fetch_world_from_service(node)
            WorldSynchronizer(_world=self.world, node=node)

        cas = self.get_cas()
        world_T_camera = get_world_to_camera_transform_matrix(cas)
        image_height, image_width = cas.get(CASViews.COLOR_IMAGE).shape[:2]
        seat_regions = []
        for furniture in self.world.get_semantic_annotations_by_type(HasSeatPlaces):
            for seat_place in furniture.seat_places:
                box = region_obb_in_camera_coordinates(
                    self.world, seat_place, world_T_camera
                )
                pixels, in_front = project_points_to_image(
                    cas, np.asarray(box.get_box_points())
                )
                if not in_front.all():
                    continue
                left, top = np.maximum(pixels.min(axis=0), 0)
                right, bottom = np.minimum(
                    pixels.max(axis=0), [image_width, image_height]
                )
                if left >= right or top >= bottom:
                    continue
                seat_region = WorldSeatRegion(
                    source=self.name,
                    furniture=SeatFurniture.CHAIR
                    if isinstance(furniture, Chair)
                    else SeatFurniture.SOFA,
                    seat_place_name=seat_place.name.name,
                )
                seat_region.roi.roi = Rect.from_corner_points(left, top, right, bottom)
                seat_region.annotations.append(
                    PositionAnnotation(
                        source=self.name,
                        translation=[float(value) for value in box.get_center()],
                    )
                )
                seat_regions.append(seat_region)
        cas.annotations.extend(seat_regions)
        self.feedback_message = f"{len(seat_regions)} seat places in view"
        return Status.SUCCESS
