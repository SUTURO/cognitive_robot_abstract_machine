"""
Subtree segmenting a T-shirt with open-vocabulary detection, without a picture of the
empty workspace.
"""

from __future__ import annotations

from py_trees.composites import Sequence

from robokudo.analysis_engine import SubtreeInterface
from robokudo.annotators.cloth_outline import ClothOutlineAnnotator
from robokudo.annotators.open_vocabulary_object_detection_annotator import (
    OpenVocabularyObjectDetectionAnnotator,
)

# %% subtree


class Subtree(SubtreeInterface):
    """
    Detects T-shirts with OWLv2, segments them with Segment Anything and annotates the
    most confident one as the cloth.
    """

    def name(self) -> str:
        return "cloth_segmentation"

    def implementation(self) -> Sequence:
        outline = ClothOutlineAnnotator()

        detection_descriptor = OpenVocabularyObjectDetectionAnnotator.Descriptor()
        detection_descriptor.parameters.classes = (
            outline.descriptor.parameters.cloth_class_names
        )
        detection_descriptor.parameters.precision_mode = True
        detection_descriptor.parameters.refine_bounding_boxes = True

        sequence = Sequence("Cloth Segmentation", memory=True)
        sequence.add_children(
            [
                OpenVocabularyObjectDetectionAnnotator(
                    name="ClothDetection", descriptor=detection_descriptor
                ),
                outline,
            ]
        )
        return sequence
