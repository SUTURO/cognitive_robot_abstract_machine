"""
YOLO object detection on request, answering with the semantic type and pose of each
detected object and saying them.
"""

# %% imports
import os
import sys
from dataclasses import dataclass

import py_trees.composites
import py_trees.decorators
from py_trees.behaviour import Behaviour

from robokudo.analysis_engine import AnalysisEngineInterface
from robokudo.annotators.cas_check import CASCheckOHExists
from robokudo.annotators.cluster_pose_bb import ClusterPoseBBAnnotator
from robokudo.annotators.collection_reader import CollectionReaderAnnotator
from robokudo.annotators.detected_objects_announcer import DetectedObjectsAnnouncer
from robokudo.annotators.gpsr.type_mapping_annotator import (
    SemanticTypeReplacementAnnotator,
)
from robokudo.annotators.image_preprocessor import ImagePreprocessorAnnotator
from robokudo.annotators.outlier_removal_objecthypothesis import (
    OutlierRemovalOnObjectHypothesisAnnotator,
)
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
    """
    Detect objects in the current frame of the HSR head camera for each query, reply
    with their semantic types and poses and publish a sentence naming them for the text-
    to-speech.
    """

    def name(self) -> str:
        """
        :return: The name identifier of this analysis engine.
        """
        return "live_yolo"

    def implementation(self) -> Pipeline:
        """
        :return: The pipeline that waits for a query, detects the objects in one camera
            image, gives them their semantic types and poses, says them and answers with
            them.
        """
        camera = CollectionReaderDescriptorFactory.create_descriptor("hsr_ros2")
        yolo = YoloAnnotator.Descriptor()
        yolo.parameters.pretrained_model = "yolov8m.pt"
        yolo.parameters.threshold = 0.6
        yolo.parameters.ignored_class_names = ["person", "dining table"]

        pipeline = Pipeline("LiveYOLO")
        pipeline.add_children(
            [
                pipeline_init(),
                QueryAnnotator(),
                CollectionReaderAnnotator(descriptor=camera),
                ImagePreprocessorAnnotator(),
                YoloAnnotator(descriptor=yolo),
                SemanticTypeReplacementAnnotator(),
                self.object_localization(),
                DetectedObjectsAnnouncer(),
                GenerateQueryResult(),
                ActionServerCheck(),
            ]
        )
        return pipeline

    def object_localization(self) -> Behaviour:
        """
        :return: The behaviour giving every detected object the pose of the largest
            cluster of its depth points, which succeeds without poses when nothing was
            detected.
        """
        localization = py_trees.composites.Sequence("object_localization", memory=True)
        localization.add_children(
            [
                CASCheckOHExists(name="ObjectHypothesesFound"),
                OutlierRemovalOnObjectHypothesisAnnotator(),
                ClusterPoseBBAnnotator(),
            ]
        )
        # the outlier removal raises without object hypotheses, so an empty scene skips it
        nothing_detected = py_trees.decorators.Inverter(
            name="NoObjectHypotheses",
            child=CASCheckOHExists(name="AnyObjectHypothesis"),
        )
        localization_unless_empty = py_trees.composites.Selector(
            "object_localization_unless_empty", memory=True
        )
        localization_unless_empty.add_children([localization, nothing_detected])
        return localization_unless_empty


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
