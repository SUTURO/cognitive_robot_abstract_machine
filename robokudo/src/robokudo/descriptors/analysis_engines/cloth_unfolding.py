"""
Analysis engine perceiving a T-shirt while it is unfolded, from a RealSense camera.
"""

from __future__ import annotations

from robokudo.analysis_engine import AnalysisEngineInterface
from robokudo.annotators.cloth_grasp_points import ClothGraspPointAnnotator
from robokudo.annotators.cloth_state_connector import ClothStateConnector
from robokudo.annotators.cloth_stretch import ClothStretchAnnotator
from robokudo.annotators.collection_reader import CollectionReaderAnnotator
from robokudo.annotators.image_preprocessor import ImagePreprocessorAnnotator
from robokudo.descriptors.factories.cr_descriptor_factory import (
    CollectionReaderDescriptorFactory,
)
from robokudo.descriptors.analysis_engines.subtree_cloth_fold_detection import (
    Subtree as ClothFoldDetection,
)
from robokudo.descriptors.analysis_engines.subtree_cloth_segmentation import (
    Subtree as ClothSegmentation,
)
from robokudo.idioms import pipeline_init
from robokudo.pipeline import Pipeline

# %% analysis engine


class AnalysisEngine(AnalysisEngineInterface):
    """
    Segments the T-shirt, finds its grasp points, its fold and how straight it is
    stretched, and records them on the T-shirt in the world.
    """

    def name(self) -> str:
        return "cloth_unfolding"

    def implementation(self) -> Pipeline:
        pipeline = Pipeline("ClothUnfoldingPipeline")
        pipeline.add_children(
            [
                pipeline_init(),
                CollectionReaderAnnotator(
                    descriptor=CollectionReaderDescriptorFactory.create_descriptor(
                        "realsense"
                    )
                ),
                ImagePreprocessorAnnotator("ImagePreprocessor"),
                ClothSegmentation().implementation(),
                ClothGraspPointAnnotator(),
                ClothFoldDetection().implementation(),
                ClothStretchAnnotator(),
                ClothStateConnector(),
            ]
        )
        return pipeline
