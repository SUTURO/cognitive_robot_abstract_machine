"""
YOLO object detection on request, answering with the detected objects and saying them.
"""

from __future__ import annotations

# %% imports
from dataclasses import dataclass

from robokudo.analysis_engine import AnalysisEngineInterface
from robokudo.annotators.collection_reader import CollectionReaderAnnotator
from robokudo.annotators.detected_objects_announcer import DetectedObjectsAnnouncer
from robokudo.annotators.image_preprocessor import ImagePreprocessorAnnotator
from robokudo.annotators.query import GenerateQueryResult, QueryAnnotator
from robokudo.annotators.yolo_annotator import YoloAnnotator
from robokudo.behaviours.action_server_checks import ActionServerCheck
from robokudo.descriptors.factories.cr_descriptor_factory import (
    CollectionReaderDescriptorFactory,
)
from robokudo.idioms import pipeline_init
from robokudo.pipeline import Pipeline


# %% analysis engine
@dataclass(kw_only=True)
class AnalysisEngine(AnalysisEngineInterface):
    """
    Detect objects in the current frame of the HSR head camera for each query, reply
    with them and publish a sentence naming them for the text-to-speech.
    """

    pretrained_model: str = "ultimate_model_v8_1.26.2.pt"
    """
    YOLO model file inside RoboKudo's models directory.
    """

    detection_threshold: float = 0.6
    """
    Minimum confidence of a detection to be answered and said.
    """

    def name(self) -> str:
        """
        :return: The name identifier of this analysis engine.
        """
        return "live_yolo"

    def implementation(self) -> Pipeline:
        """
        :return: The pipeline that waits for a query, detects the objects in one camera
            image, says them and answers with them.
        """
        camera = CollectionReaderDescriptorFactory.create_descriptor("hsr_ros2")
        yolo = YoloAnnotator.Descriptor()
        yolo.parameters.pretrained_model = self.pretrained_model
        yolo.parameters.threshold = self.detection_threshold

        pipeline = Pipeline("LiveYOLO")
        pipeline.add_children(
            [
                pipeline_init(),
                QueryAnnotator(),
                CollectionReaderAnnotator(descriptor=camera),
                ImagePreprocessorAnnotator(),
                YoloAnnotator(descriptor=yolo),
                DetectedObjectsAnnouncer(),
                GenerateQueryResult(),
                ActionServerCheck(),
            ]
        )
        return pipeline
