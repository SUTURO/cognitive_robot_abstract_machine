"""
Analysis engine answering object queries with RoboKudo's pretrained YOLO model.

Unlike the GPSR object detection, it needs no team specific weights: the model is one
ultralytics provides, downloaded into RoboKudo's ``models`` directory on first use, so
it detects the classes of the COCO dataset (e.g. ``bottle``, ``cup``, ``banana``).

The pipeline implements the following functionality:

* Waiting for a query on the ``robokudo/query`` action
* Reading the HSR head camera
* Detecting objects with YOLO
* Answering the query with the class of every detected object

The answer names the objects but does not localize them: estimating poses from the
depth image can fail for a single detection and would then fail the whole query.
"""

from __future__ import annotations

# %% imports

from robokudo.analysis_engine import AnalysisEngineInterface
from robokudo.annotators.collection_reader import CollectionReaderAnnotator
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


class AnalysisEngine(AnalysisEngineInterface):
    """
    Answers every query with the objects the pretrained YOLO model detects.
    """

    def name(self) -> str:
        """
        :return: The name identifier of this analysis engine.
        """
        return "yolo_object_detection"

    def implementation(self) -> Pipeline:
        """
        :return: The pipeline that waits for a query, detects the objects in the
            current camera image and answers with them.
        """
        camera_config = CollectionReaderDescriptorFactory.create_descriptor("hsr_ros2")

        pipeline = Pipeline("YoloObjectDetectionPipeline")
        pipeline.add_children(
            [
                pipeline_init(),
                QueryAnnotator(),
                CollectionReaderAnnotator(descriptor=camera_config),
                ImagePreprocessorAnnotator("ImagePreprocessor"),
                YoloAnnotator(descriptor=self.detection_descriptor()),
                GenerateQueryResult(),
                ActionServerCheck(),
            ]
        )
        return pipeline

    @staticmethod
    def detection_descriptor() -> YoloAnnotator.Descriptor:
        """
        :return: The configuration of the pretrained YOLO model.
        """
        descriptor = YoloAnnotator.Descriptor()
        descriptor.parameters.pretrained_model = "yolov8m.pt"
        descriptor.parameters.threshold = 0.6
        # People and furniture are not the objects a scene report should list.
        descriptor.parameters.ignored_class_names = ["person", "dining table"]
        return descriptor
