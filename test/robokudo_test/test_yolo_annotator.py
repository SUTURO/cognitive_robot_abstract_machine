"""
Tests for which image the YOLO annotator detects objects in.
"""

from __future__ import annotations

import numpy as np
import pytest

import robokudo.pipeline
from robokudo.annotators.yolo_annotator import YoloAnnotator
from robokudo.cas import CAS, CASViews
from robokudo.pipeline import Pipeline
from robokudo.types.cv import ColorToDepthRatio

# %% detection image


@pytest.mark.parametrize(
    "image_view", [CASViews.COLOR_IMAGE, CASViews.HEIGHT_COLORED_IMAGE]
)
def test_objects_are_detected_in_the_configured_image(image_view: CASViews) -> None:
    """
    The model is given the image of the configured view, not always the color image.
    """
    descriptor = YoloAnnotator.Descriptor()
    descriptor.parameters.pretrained_model = "unloaded.pt"
    descriptor.parameters.image_view = image_view
    annotator = YoloAnnotator(descriptor=descriptor)
    pipeline = Pipeline()
    pipeline.add_child(annotator)
    pipeline.cas = CAS()
    pipeline.cas.set(CASViews.COLOR_TO_DEPTH_RATIO, ColorToDepthRatio())
    images = {
        view: np.full((4, 4, 3), value, dtype=np.uint8)
        for value, view in enumerate(
            [CASViews.COLOR_IMAGE, CASViews.HEIGHT_COLORED_IMAGE]
        )
    }
    for view, image in images.items():
        pipeline.cas.set(view, image)

    assert (annotator._detection_image() == images[image_view]).all()
