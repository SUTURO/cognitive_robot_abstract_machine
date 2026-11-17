"""
Location of a fold detected in a cloth lying on a surface, and of where to lay its
folded-over part to unfold it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np
import numpy.typing as npt
from py_trees.common import Status

from krrood_spatial_math.spatial_types import Point3
from robokudo.annotators.core import BaseAnnotator
from robokudo.cas import CAS, CASViews
from robokudo.types.annotation import Classification
from robokudo.types.camera import RegisteredDepthImage
from robokudo.types.cloth import ClothFoldAnnotation, ClothHypothesis
from robokudo.types.cv import Point2D, Rect
from robokudo.types.scene import ObjectHypothesis
from semantic_digital_twin.world_description.cloth import ClothFold

# %% detections


@dataclass
class FoldRegion:
    """
    A region of the color image a fold was detected in.
    """

    region: Rect
    """
    The bounding box of the folded-over part of the cloth.
    """

    confidence: float
    """
    How confident the detector is that the region shows a fold.
    """

    @property
    def center(self) -> Point2D:
        """
        The pixel in the middle of the region.
        """
        return Point2D(
            x=self.region.pos.x + self.region.width // 2,
            y=self.region.pos.y + self.region.height // 2,
        )


# %% annotator


class ClothFoldAnnotator(BaseAnnotator):
    """
    Annotates each cloth hypothesis whose heights are measured with the most confident
    fold detected inside it.

    The fold opens towards the direction in which the cloth is highest within the fold
    region; the folded-over part is laid down as far beyond the cloth's outline in that
    direction as the outline is from the fold's middle.
    """

    class Descriptor(BaseAnnotator.Descriptor):
        """
        Descriptor for the ClothFoldAnnotator.
        """

        class Parameters:
            """
            Parameters for the ClothFoldAnnotator.
            """

            def __init__(self) -> None:
                self.fold_class_name: str = "fold"
                """
                The class name of the object hypotheses that are detected folds.
                """
                self.direction_count: int = 24
                """
                How many directions, evenly spread around the fold's middle, are
                searched for the one the fold opens towards.
                """
                self.region_margin: int = 5
                """
                How many pixels beyond the detected region the search for the direction
                looks.
                """

    def __init__(
        self,
        name: str = "ClothFoldAnnotator",
        descriptor: ClothFoldAnnotator.Descriptor | None = None,
    ) -> None:
        super().__init__(name, descriptor)

    def update(self) -> Status:
        """
        Annotate each cloth hypothesis with the fold detected in it, if any.
        """
        cas = self.get_cas()
        depth = RegisteredDepthImage.from_cas(cas)
        fold_regions = self.detected_fold_regions()
        visualization_image = cas.get_copy(CASViews.COLOR_IMAGE)
        for hypothesis in cas.filter_annotations_by_type(ClothHypothesis):
            fold = self._locate_fold(hypothesis, fold_regions, depth)
            if fold is None:
                continue
            hypothesis.annotations.append(fold)
            left, top, right, bottom = fold.fold_region.get_corner_points()
            cv2.rectangle(
                visualization_image, (left, top), (right, bottom), (0, 255, 0), 2
            )
            cv2.circle(
                visualization_image,
                (fold.edge_pixel.x, fold.edge_pixel.y),
                5,
                (0, 255, 255),
                -1,
            )
        self.get_annotator_output_struct().set_image(visualization_image)
        return Status.SUCCESS

    def detected_fold_regions(self) -> list[FoldRegion]:
        """
        :return: The regions of the object hypotheses classified as folds, each with
            the confidence of its most confident fold classification.
        """
        fold_class_name = self.descriptor.parameters.fold_class_name
        fold_regions = []
        for hypothesis in self.get_cas().filter_annotations_by_type(ObjectHypothesis):
            confidences = [
                classification.confidence
                for classification in CAS.filter_by_type(
                    Classification, hypothesis.annotations
                )
                if classification.classname == fold_class_name
            ]
            if len(confidences) > 0:
                fold_regions.append(
                    FoldRegion(region=hypothesis.roi.roi, confidence=max(confidences))
                )
        return fold_regions

    def _locate_fold(
        self,
        hypothesis: ClothHypothesis,
        fold_regions: list[FoldRegion],
        depth: RegisteredDepthImage,
    ) -> ClothFoldAnnotation | None:
        """
        :return: The most confident fold whose middle lies on the cloth, or ``None`` if
            there is none or the camera measured no depth where it has to be located.
        """
        if hypothesis.heights is None:
            return None
        folds_on_cloth = [
            fold_region
            for fold_region in fold_regions
            if cv2.pointPolygonTest(
                hypothesis.contour,
                (fold_region.center.x, fold_region.center.y),
                False,
            )
            >= 0
        ]
        if len(folds_on_cloth) == 0:
            return None
        fold_region = max(folds_on_cloth, key=lambda region: region.confidence)

        direction = self.opening_direction(fold_region, hypothesis.heights)
        if direction is None:
            return None
        edge_pixel = self.edge_pixel(
            fold_region.center, direction, hypothesis.contour, hypothesis.heights.shape
        )
        if edge_pixel is None:
            return None
        fold_point = depth.world_point(fold_region.center)
        edge_point = depth.world_point(edge_pixel)
        if fold_point is None or edge_point is None:
            return None
        unfold_target = Point3(
            *(2 * edge_point.to_np()[:3] - fold_point.to_np()[:3]),
            reference_frame=fold_point.reference_frame,
        )
        return ClothFoldAnnotation(
            source=self.name,
            fold_region=fold_region.region,
            confidence=fold_region.confidence,
            edge_pixel=edge_pixel,
            fold=ClothFold(fold_point=fold_point, unfold_target=unfold_target),
        )

    def opening_direction(
        self, fold_region: FoldRegion, height_image: npt.NDArray[np.float64]
    ) -> float | None:
        """
        :param fold_region: Where the fold was detected.
        :param height_image: The world height of the cloth at each pixel.
        :return: The angle in radians, in image coordinates, of the direction from the
            fold's middle along which the cloth is highest on average within the
            region, or ``None`` if no height was measured in the region.
        """
        parameters = self.descriptor.parameters
        margin = parameters.region_margin
        left, top, right, bottom = fold_region.region.get_corner_points()
        image_height, image_width = height_image.shape
        left, top = max(left - margin, 0), max(top - margin, 0)
        right = min(right + margin, image_width - 1)
        bottom = min(bottom + margin, image_height - 1)
        center = fold_region.center
        longest_ray = math.ceil(math.hypot(right - left, bottom - top))
        steps = np.arange(longest_ray + 1)

        mean_heights = {}
        for angle in np.linspace(
            0, 2 * math.pi, parameters.direction_count, endpoint=False
        ):
            columns = (center.x + steps * math.cos(angle)).astype(int)
            rows = (center.y + steps * math.sin(angle)).astype(int)
            inside = (
                (columns >= left)
                & (columns <= right)
                & (rows >= top)
                & (rows <= bottom)
            )
            heights = height_image[rows[inside], columns[inside]]
            heights = heights[~np.isnan(heights)]
            if len(heights) > 0:
                mean_heights[float(angle)] = heights.mean()
        if len(mean_heights) == 0:
            return None
        return max(mean_heights, key=mean_heights.get)

    @staticmethod
    def edge_pixel(
        start: Point2D,
        direction: float,
        contour: npt.NDArray[np.int32],
        image_shape: tuple[int, int],
    ) -> Point2D | None:
        """
        :param start: A pixel inside the cloth.
        :param direction: The angle in radians, in image coordinates, to walk in.
        :param contour: The outline of the cloth.
        :param image_shape: The height and width of the color image.
        :return: The first pixel walking from the start in the direction that is not
            inside the outline, or ``None`` if the walk leaves the image first.
        """
        image_height, image_width = image_shape
        for step in range(1, max(image_height, image_width)):
            column = int(round(start.x + step * math.cos(direction)))
            row = int(round(start.y + step * math.sin(direction)))
            if not (0 <= column < image_width and 0 <= row < image_height):
                return None
            if cv2.pointPolygonTest(contour, (column, row), False) <= 0:
                return Point2D(x=column, y=row)
        return None
