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
    import numpy.typing as npt
    from semantic_digital_twin.world import World
    from semantic_digital_twin.world_description.world_entity import Region

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

        seat_regions = []
        for furniture in self.world.get_semantic_annotations_by_type(HasSeatPlaces):
            for seat_place in furniture.seat_places:
                seat_region = self.seat_region_in_view(seat_place, furniture)
                if seat_region is not None:
                    seat_regions.append(seat_region)
        self.get_cas().annotations.extend(seat_regions)
        self.feedback_message = f"{len(seat_regions)} seat places in view"
        return Status.SUCCESS

    def seat_region_in_view(
        self, seat_place: Region, furniture: HasSeatPlaces
    ) -> WorldSeatRegion | None:
        """
        :param seat_place: A seat place of the shared world.
        :param furniture: The furniture the seat place belongs to.
        :return: The seat region of the seat place, or None if the seat place is not
            entirely in front of the camera or lies outside the color image.
        """
        cas = self.get_cas()
        box = region_obb_in_camera_coordinates(
            self.world, seat_place, get_world_to_camera_transform_matrix(cas)
        )
        image_region = self.image_region_of(np.asarray(box.get_box_points()))
        if image_region is None:
            return None

        seat_region = WorldSeatRegion(
            source=self.name,
            furniture=SeatFurniture.CHAIR
            if isinstance(furniture, Chair)
            else SeatFurniture.SOFA,
            seat_place_name=seat_place.name.name,
        )
        seat_region.roi.roi = image_region
        seat_region.annotations.append(
            PositionAnnotation(
                source=self.name,
                translation=[float(coordinate) for coordinate in box.get_center()],
            )
        )
        return seat_region

    def image_region_of(self, corners: npt.NDArray[np.float64]) -> Rect | None:
        """
        :param corners: The corners of a box in the camera frame, one per row.
        :return: The part of the color image the box covers, or None if a corner lies
            behind the camera or the box misses the image.
        """
        cas = self.get_cas()
        pixels, in_front = project_points_to_image(cas, corners)
        if not in_front.all():
            return None

        image_height, image_width = cas.get(CASViews.COLOR_IMAGE).shape[:2]
        left, top = np.maximum(pixels.min(axis=0), 0)
        right, bottom = np.minimum(pixels.max(axis=0), [image_width, image_height])
        if left >= right or top >= bottom:
            return None
        return Rect.from_corner_points(left, top, right, bottom)
