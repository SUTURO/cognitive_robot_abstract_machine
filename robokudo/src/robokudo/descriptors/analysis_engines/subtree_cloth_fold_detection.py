"""
Subtree detecting the fold in a segmented cloth, with the fold detection model of
``robokudo_suturo``.
"""

from __future__ import annotations

from py_trees.composites import Sequence

from robokudo.analysis_engine import SubtreeInterface
from robokudo.annotators.cloth_fold_detection import ClothFoldAnnotator
from robokudo.annotators.cloth_height_coloring import ClothHeightColoringAnnotator
from robokudo.annotators.yolo_annotator import YoloAnnotator
from robokudo.cas import CASViews

# %% subtree


class Subtree(SubtreeInterface):
    """
    Colors each cloth hypothesis by height, detects folds in that image with YOLO and
    locates the most confident fold on each cloth.
    """

    def name(self) -> str:
        return "cloth_fold_detection"

    def implementation(self) -> Sequence:
        yolo_descriptor = YoloAnnotator.Descriptor()
        yolo_descriptor.parameters.ros_package_name = "robokudo_suturo"
        yolo_descriptor.parameters.weights_path = "weights/cloth_fold.pt"
        yolo_descriptor.parameters.class_names_path = (
            "data/json/id2name_cloth_fold.json"
        )
        yolo_descriptor.parameters.image_view = CASViews.HEIGHT_COLORED_IMAGE
        yolo_descriptor.parameters.threshold = 0.25

        sequence = Sequence("Cloth Fold Detection", memory=True)
        sequence.add_children(
            [
                ClothHeightColoringAnnotator(),
                YoloAnnotator(name="ClothFoldYolo", descriptor=yolo_descriptor),
                ClothFoldAnnotator(),
            ]
        )
        return sequence
