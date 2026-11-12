"""
YOLO object detection on request, answering with the detected objects and saying them.
"""

# %% imports
import os
import sys
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
@dataclass
class AnalysisEngine(AnalysisEngineInterface):
    """Detect objects in the current frame of the HSR head camera for each
    query, reply with them and publish a sentence naming them for the
    text-to-speech."""

    def name(self) -> str:
        return "live_yolo"

    def implementation(self) -> Pipeline:
        camera = CollectionReaderDescriptorFactory.create_descriptor("hsr_ros2")
        yolo = YoloAnnotator.Descriptor()
        yolo.parameters.pretrained_model = "ultimate_model_v8_1.26.2.pt"
        yolo.parameters.threshold = 0.6

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


# %% direct start
def main() -> None:
    """
    Start RoboKudo with the YOLO object detection analysis engine.
    """
    os.execv(
        sys.executable,
        [
            sys.executable,
            "-m",
            "robokudo.scripts.main",
            "_ae",
            "yolo_object_detection",
            *sys.argv[1:],
        ],
    )


if __name__ == "__main__":
    main()
