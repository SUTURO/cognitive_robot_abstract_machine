"""
Coloring of a cloth by its height, as the fold detection model expects to see it.
"""

from __future__ import annotations

import cv2
import numpy as np
import numpy.typing as npt
from krrood.ormatic.utils import classproperty
from py_trees.common import Status

from robokudo.annotators.core import BaseAnnotator
from robokudo.cas import CASViews
from robokudo.types.camera import RegisteredDepthImage
from robokudo.types.cloth import ClothHypothesis

# %% annotator


class ClothHeightColoringAnnotator(BaseAnnotator):
    """
    Measures the height of each cloth hypothesis at each of its pixels and stores the
    color image with the cloths colored by height as
    :attr:`~robokudo.cas.CASViews.HEIGHT_COLORED_IMAGE`.
    """

    @classproperty
    def mask_foreground(cls) -> int:
        """
        Value of the cloth pixels in the masks this annotator computes.
        """
        return 255

    def __init__(
        self,
        name: str = "ClothHeightColoringAnnotator",
        descriptor: BaseAnnotator.Descriptor | None = None,
    ) -> None:
        super().__init__(name, descriptor)

    def update(self) -> Status:
        """
        Measure the heights of each cloth and store the height colored image.
        """
        cas = self.get_cas()
        depth = RegisteredDepthImage.from_cas(cas)
        height_colored_image = cas.get_copy(CASViews.COLOR_IMAGE)
        for hypothesis in cas.filter_annotations_by_type(ClothHypothesis):
            hypothesis.heights = self.height_image(
                hypothesis.contour, height_colored_image.shape[:2], depth
            )
            self.color_by_height(height_colored_image, hypothesis.heights)
        cas.set(CASViews.HEIGHT_COLORED_IMAGE, height_colored_image)
        self.get_annotator_output_struct().set_image(height_colored_image)
        return Status.SUCCESS

    def height_image(
        self,
        contour: npt.NDArray[np.int32],
        image_shape: tuple[int, int],
        depth: RegisteredDepthImage,
    ) -> npt.NDArray[np.float64]:
        """
        :param contour: The outline of the cloth.
        :param image_shape: The height and width of the color image.
        :param depth: The depth image the heights are measured from.
        :return: The world height of the cloth at each color image pixel, ``NaN``
            outside the cloth and where the camera measured no depth.
        """
        mask = np.zeros(image_shape, dtype=np.uint8)
        cv2.drawContours(mask, [contour], -1, self.mask_foreground, cv2.FILLED)
        rows, columns = np.nonzero(mask == self.mask_foreground)
        heights = np.full(image_shape, np.nan)
        heights[rows, columns] = depth.world_points(np.column_stack([columns, rows]))[
            :, 2
        ]
        return heights

    @staticmethod
    def color_by_height(
        image: npt.NDArray[np.uint8], heights: npt.NDArray[np.float64]
    ) -> None:
        """
        Color each pixel with a measured height from green at the lowest to blue at the
        highest, leaving the other pixels as they are.

        :param image: The image to color, in BGR order.
        :param heights: The height at each pixel of the image.
        """
        measured = ~np.isnan(heights)
        if not measured.any():
            return
        measured_heights = heights[measured]
        lowest = measured_heights.min()
        height_range = measured_heights.max() - lowest
        ratios = (measured_heights - lowest) / (
            height_range if height_range > 0 else 1.0
        )
        image[measured] = np.column_stack(
            [ratios * 255, (1 - ratios) * 255, np.zeros_like(ratios)]
        ).astype(np.uint8)
