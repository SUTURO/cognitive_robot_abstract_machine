"""
The outline of a cloth an object detector found and segmented.
"""

from __future__ import annotations

import cv2
import numpy as np
import numpy.typing as npt
from krrood.ormatic.utils import classproperty
from py_trees.common import Status

from robokudo.annotators.core import BaseAnnotator
from robokudo.annotators.exceptions import ClothMaskMissing
from robokudo.cas import CAS, CASViews
from robokudo.types.annotation import Classification
from robokudo.types.cloth import ClothHypothesis
from robokudo.types.scene import ObjectHypothesis

# %% annotator


class ClothOutlineAnnotator(BaseAnnotator):
    """
    Annotates the object hypothesis most confidently classified as a cloth as a cloth
    hypothesis, outlined by the largest region of its mask.
    """

    class Descriptor(BaseAnnotator.Descriptor):
        """
        Descriptor for the ClothOutlineAnnotator.
        """

        class Parameters:
            """
            Parameters for the ClothOutlineAnnotator.
            """

            def __init__(self) -> None:
                self.cloth_class_names: list[str] = ["t-shirt"]
                """
                The class names of the object hypotheses that are cloths.
                """

    @classproperty
    def mask_foreground(cls) -> int:
        """
        Value of the object pixels in the masks of the object hypotheses.
        """
        return 255

    def __init__(
        self,
        name: str = "ClothOutlineAnnotator",
        descriptor: ClothOutlineAnnotator.Descriptor | None = None,
    ) -> None:
        super().__init__(name, descriptor)

    def update(self) -> Status:
        """
        Annotate the most confidently detected cloth as a cloth hypothesis, if one was
        detected.

        :raises ClothMaskMissing: If the detected cloth was not segmented.
        """
        cas = self.get_cas()
        detections = [
            hypothesis
            for hypothesis in cas.filter_annotations_by_type(ObjectHypothesis)
            if not isinstance(hypothesis, ClothHypothesis)
            and self.cloth_confidence(hypothesis) is not None
        ]
        if len(detections) == 0:
            self.feedback_message = "No cloth detected"
            return Status.SUCCESS
        detection = max(detections, key=self.cloth_confidence)
        if detection.roi.mask is None:
            raise ClothMaskMissing(annotator_name=detection.source)
        outline = self.outline(detection)
        if outline is None:
            self.feedback_message = "The detected cloth's mask is empty"
            return Status.SUCCESS

        cloth = ClothHypothesis(contour=outline)
        cloth.source = self.name
        cloth.roi = detection.roi
        cloth.annotations.extend(
            CAS.filter_by_type(Classification, detection.annotations)
        )
        cas.annotations.append(cloth)

        visualization_image = cas.get_copy(CASViews.COLOR_IMAGE)
        cv2.drawContours(visualization_image, [cloth.contour], -1, (0, 255, 0), 2)
        self.get_annotator_output_struct().set_image(visualization_image)
        return Status.SUCCESS

    def cloth_confidence(self, hypothesis: ObjectHypothesis) -> float | None:
        """
        :return: The confidence of the hypothesis's most confident classification as a
            cloth, or ``None`` if it is not classified as one.
        """
        cloth_class_names = self.descriptor.parameters.cloth_class_names
        confidences = [
            classification.confidence
            for classification in CAS.filter_by_type(
                Classification, hypothesis.annotations
            )
            if classification.classname in cloth_class_names
        ]
        return max(confidences) if len(confidences) > 0 else None

    def outline(self, detection: ObjectHypothesis) -> npt.NDArray[np.int32] | None:
        """
        :param detection: A segmented object hypothesis.
        :return: The outline of the largest region of its mask, in color image pixels,
            or ``None`` if the mask is empty.
        """
        contours, _ = cv2.findContours(
            (detection.roi.mask == self.mask_foreground).astype(np.uint8),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
            offset=(detection.roi.roi.pos.x, detection.roi.roi.pos.y),
        )
        if len(contours) == 0:
            return None
        return max(contours, key=cv2.contourArea)
