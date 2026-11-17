"""
Whether a cloth held up by two grippers at its grasp points is pulled straight between
them.
"""

from __future__ import annotations

import cv2
import numpy as np
import numpy.typing as npt
from py_trees.common import Status

from robokudo.annotators.core import BaseAnnotator
from robokudo.cas import CAS, CASViews
from robokudo.types.cloth import (
    ClothGraspPointsAnnotation,
    ClothHypothesis,
    ClothStretchAnnotation,
)

# %% annotator


class ClothStretchAnnotator(BaseAnnotator):
    """
    Annotates each cloth hypothesis held at its grasp points with how far the edge
    between the grippers strays from a straight line.

    Of the two ways along the outline between the grippers, the shorter is the edge.
    """

    class Descriptor(BaseAnnotator.Descriptor):
        """
        Descriptor for the ClothStretchAnnotator.
        """

        class Parameters:
            """
            Parameters for the ClothStretchAnnotator.
            """

            def __init__(self) -> None:
                self.gripper_clearance: int = 100
                """
                How many pixels inwards from the grasp points the edge is measured from,
                so that the grippers holding the cloth are left out.
                """
                self.outline_simplification: float = 0.01
                """
                How far the simplified outline may stray from the traced one, as a
                fraction of the outline's length.
                """
                self.maximum_deviation: float = 5.0
                """
                How many pixels the edge may stray from a straight line for the cloth to
                count as stretched.
                """

    def __init__(
        self,
        name: str = "ClothStretchAnnotator",
        descriptor: ClothStretchAnnotator.Descriptor | None = None,
    ) -> None:
        super().__init__(name, descriptor)

    def update(self) -> Status:
        """
        Annotate each cloth hypothesis with grasp points with how straight its edge
        between the grippers is.
        """
        visualization_image = self.get_cas().get_copy(CASViews.COLOR_IMAGE)
        for hypothesis in self.get_cas().filter_annotations_by_type(ClothHypothesis):
            grasp_points = CAS.filter_by_type(
                ClothGraspPointsAnnotation, hypothesis.annotations
            )
            if len(grasp_points) == 0:
                continue
            edge = self.edge_between_grippers(hypothesis.contour, grasp_points[-1])
            maximum_deviation = self.maximum_deviation_from_straight_line(edge)
            hypothesis.annotations.append(
                ClothStretchAnnotation(
                    source=self.name,
                    maximum_deviation=maximum_deviation,
                    is_stretched=maximum_deviation
                    < self.descriptor.parameters.maximum_deviation,
                )
            )
            cv2.polylines(
                visualization_image,
                [edge.reshape(-1, 1, 2)],
                isClosed=False,
                color=(0, 255, 0),
                thickness=2,
            )
        self.get_annotator_output_struct().set_image(visualization_image)
        return Status.SUCCESS

    def edge_between_grippers(
        self,
        contour: npt.NDArray[np.int32],
        grasp_points: ClothGraspPointsAnnotation,
    ) -> npt.NDArray[np.int32]:
        """
        :param contour: The outline of a cloth held up at its grasp points.
        :param grasp_points: Where the grippers hold the cloth.
        :return: The vertices of the simplified outline along the edge between the
            grippers, one (x, y) per row, in order along the outline.
        """
        parameters = self.descriptor.parameters
        outline = cv2.approxPolyDP(
            contour,
            parameters.outline_simplification * cv2.arcLength(contour, True),
            True,
        ).reshape(-1, 2)
        left_pixel, right_pixel = grasp_points.left_pixel, grasp_points.right_pixel
        left_end = (left_pixel.x + parameters.gripper_clearance, left_pixel.y)
        right_end = (right_pixel.x - parameters.gripper_clearance, right_pixel.y)
        first, last = sorted(
            int(np.linalg.norm(outline - end, axis=1).argmin())
            for end in (left_end, right_end)
        )
        inner_way = outline[first : last + 1]
        outer_way = np.concatenate([outline[last:], outline[: first + 1]])
        return min(
            inner_way,
            outer_way,
            key=lambda way: cv2.arcLength(way.reshape(-1, 1, 2), False),
        )

    @staticmethod
    def maximum_deviation_from_straight_line(edge: npt.NDArray[np.int32]) -> float:
        """
        :param edge: Vertices along an edge, one (x, y) per row.
        :return: The largest distance in pixels of a vertex from the straight line
            through the first and last vertex, infinite if the two coincide.
        """
        start, end = edge[0].astype(float), edge[-1].astype(float)
        direction = end - start
        length = np.linalg.norm(direction)
        if length == 0:
            return float("inf")
        offsets = edge.astype(float) - start
        distances = (
            np.abs(direction[0] * offsets[:, 1] - direction[1] * offsets[:, 0]) / length
        )
        return float(distances.max())
