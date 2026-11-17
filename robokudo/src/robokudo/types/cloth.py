"""
What perception finds out about a cloth while it unfolds it.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import numpy.typing as npt

from robokudo.types.core import Annotation
from robokudo.types.cv import Point2D, Rect
from robokudo.types.scene import ObjectHypothesis
from semantic_digital_twin.world_description.cloth import ClothFold, ClothGraspPoints

# %% the cloth


@dataclass
class ClothHypothesis(ObjectHypothesis):
    """
    A cloth lying in, or held up into, a workspace that is otherwise empty.
    """

    contour: npt.NDArray[np.int32] = field(
        default_factory=lambda: np.empty((0, 1, 2), dtype=np.int32)
    )
    """
    The outline of the cloth in color image pixels, as OpenCV traces a contour.
    """

    heights: npt.NDArray[np.float64] | None = None
    """
    The world height of the cloth at each color image pixel, ``NaN`` outside the cloth
    and where the camera measured no depth, or ``None`` if not yet measured.
    """


# %% unfolding


@dataclass(kw_only=True)
class ClothGraspPointsAnnotation(Annotation):
    """
    The two points to take a cloth by to lift it and shake it out.
    """

    left_pixel: Point2D
    """
    The color image pixel of the left grasp point.
    """

    right_pixel: Point2D
    """
    The color image pixel of the right grasp point.
    """

    grasp_points: ClothGraspPoints
    """
    The grasp points in the world frame.
    """


@dataclass(kw_only=True)
class ClothFoldAnnotation(Annotation):
    """
    A fold in a cloth lying on a surface.
    """

    fold_region: Rect
    """
    The color image region the detector found the folded-over part of the cloth in.
    """

    confidence: float
    """
    How confident the detector is that the region shows a fold.
    """

    edge_pixel: Point2D
    """
    The color image pixel where the cloth's outline crosses the direction the fold
    opens in.
    """

    fold: ClothFold
    """
    The fold in the world frame.
    """


@dataclass(kw_only=True)
class ClothStretchAnnotation(Annotation):
    """
    How straight the edge of a cloth held up by two grippers is pulled.
    """

    maximum_deviation: float
    """
    How far, in color image pixels, the edge strays at most from the straight line
    between the grippers.
    """

    is_stretched: bool
    """
    Whether the edge is straight enough for the cloth to count as stretched.
    """
