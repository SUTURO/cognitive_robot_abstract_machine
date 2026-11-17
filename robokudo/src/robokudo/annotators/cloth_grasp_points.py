"""
The two points a pair of grippers takes a crumpled cloth by to lift it and shake it out.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np
import numpy.typing as npt
from py_trees.common import Status

from robokudo.annotators.core import BaseAnnotator
from robokudo.cas import CASViews
from robokudo.types.camera import RegisteredDepthImage
from robokudo.types.cloth import ClothGraspPointsAnnotation, ClothHypothesis
from robokudo.types.cv import Point2D
from semantic_digital_twin.world_description.cloth import ClothGraspPoints

# %% grasp pixels


@dataclass
class ClothGraspPixels:
    """
    The color image pixels of the two points to take a cloth by.
    """

    left_pixel: Point2D
    """
    The pixel of the grasp point on the left of the cloth.
    """

    right_pixel: Point2D
    """
    The pixel of the grasp point on the right of the cloth.
    """


# %% annotator


class ClothGraspPointAnnotator(BaseAnnotator):
    """
    Annotates each cloth hypothesis with the leftmost and rightmost points of the upper
    part of its outline, moved a little into the cloth so that a gripper closes on it.
    """

    class Descriptor(BaseAnnotator.Descriptor):
        """
        Descriptor for the ClothGraspPointAnnotator.
        """

        class Parameters:
            """
            Parameters for the ClothGraspPointAnnotator.
            """

            def __init__(self) -> None:
                self.examined_height_fraction: float = 1.0
                """
                The upper part of the cloth's bounding box, as a fraction of its height,
                whose outline the grasp points are chosen from.
                """
                self.inward_fraction: float = 0.1
                """
                How far each grasp point is moved from the outline towards the centroid
                of the examined outline, as a fraction of the distance between them.
                """

    def __init__(
        self,
        name: str = "ClothGraspPointAnnotator",
        descriptor: ClothGraspPointAnnotator.Descriptor | None = None,
    ) -> None:
        super().__init__(name, descriptor)

    def update(self) -> Status:
        """
        Annotate each cloth hypothesis with its grasp points, where the camera measured
        depth at both.
        """
        cas = self.get_cas()
        depth = RegisteredDepthImage.from_cas(cas)
        visualization_image = cas.get_copy(CASViews.COLOR_IMAGE)
        for hypothesis in cas.filter_annotations_by_type(ClothHypothesis):
            grasp_pixels = self.grasp_pixels(hypothesis.contour)
            left_grasp_point = depth.world_point(grasp_pixels.left_pixel)
            right_grasp_point = depth.world_point(grasp_pixels.right_pixel)
            if left_grasp_point is None or right_grasp_point is None:
                self.rk_logger.warning(
                    "No depth measured at a grasp point of %s", hypothesis
                )
                continue
            hypothesis.annotations.append(
                ClothGraspPointsAnnotation(
                    source=self.name,
                    left_pixel=grasp_pixels.left_pixel,
                    right_pixel=grasp_pixels.right_pixel,
                    grasp_points=ClothGraspPoints(
                        left_grasp_point=left_grasp_point,
                        right_grasp_point=right_grasp_point,
                    ),
                )
            )
            for pixel, color in (
                (grasp_pixels.left_pixel, (0, 0, 255)),
                (grasp_pixels.right_pixel, (255, 0, 0)),
            ):
                cv2.circle(visualization_image, (pixel.x, pixel.y), 5, color, -1)
        self.get_annotator_output_struct().set_image(visualization_image)
        return Status.SUCCESS

    def grasp_pixels(self, contour: npt.NDArray[np.int32]) -> ClothGraspPixels:
        """
        :param contour: The outline of a cloth, as OpenCV traces a contour.
        :return: The pixels of the grasp points on the outline's upper part.
        """
        parameters = self.descriptor.parameters
        outline = contour.reshape(-1, 2)
        top = outline[:, 1].min()
        height = outline[:, 1].max() - top
        examined = outline[
            outline[:, 1] <= top + height * parameters.examined_height_fraction
        ]
        leftmost = examined[examined[:, 0].argmin()]
        rightmost = examined[examined[:, 0].argmax()]

        moments = cv2.moments(examined.reshape(-1, 1, 2))
        centroid = (
            np.array([moments["m10"], moments["m01"]]) / moments["m00"]
            if moments["m00"] != 0
            else (leftmost + rightmost) / 2
        )

        def moved_inward(outline_point: npt.NDArray[np.int32]) -> Point2D:
            """
            :return: The pixel the outline point is moved to towards the centroid.
            """
            inward = outline_point + parameters.inward_fraction * (
                centroid - outline_point
            )
            return Point2D(x=int(inward[0]), y=int(inward[1]))

        return ClothGraspPixels(
            left_pixel=moved_inward(leftmost), right_pixel=moved_inward(rightmost)
        )
