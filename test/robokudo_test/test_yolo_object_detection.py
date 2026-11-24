"""
Tests of the analysis engine that answers queries with the objects YOLO detects in the
HSR head camera image and says them.
"""

# %% imports
from __future__ import annotations

from py_trees.behaviour import Behaviour

from robokudo.annotators.collection_reader import CollectionReaderAnnotator
from robokudo.annotators.detected_objects_announcer import DetectedObjectsAnnouncer
from robokudo.annotators.image_preprocessor import ImagePreprocessorAnnotator
from robokudo.annotators.query import GenerateQueryResult, QueryAnnotator
from robokudo.annotators.yolo_annotator import YoloAnnotator
from robokudo.behaviours.action_server_checks import ActionServerCheck
from robokudo.descriptors.analysis_engines.yolo_object_detection_announcer import (
    AnalysisEngine,
)
from robokudo.descriptors.camera_configs.config_hsr_ros2 import HsrRos2CameraConfig
from robokudo.pipeline import Pipeline


# %% helpers
def _child_of_type(pipeline: Pipeline, child_type: type[Behaviour]) -> Behaviour:
    """
    :return: The only direct child of the pipeline with the given type.
    """
    (child,) = [child for child in pipeline.children if type(child) is child_type]
    return child


# %% pipeline composition
def test_pipeline_answers_a_query_after_detecting_and_saying_objects(node):
    """
    After its initialization, the pipeline waits for a query, reads one image, detects
    the objects, says them and answers the query.
    """
    pipeline = AnalysisEngine().implementation()

    assert [type(child) for child in pipeline.children[1:]] == [
        QueryAnnotator,
        CollectionReaderAnnotator,
        ImagePreprocessorAnnotator,
        YoloAnnotator,
        DetectedObjectsAnnouncer,
        GenerateQueryResult,
        ActionServerCheck,
    ]


def test_pipeline_reads_the_hsr_head_camera(node):
    """
    The camera images come from the HSR head camera over ROS.
    """
    pipeline = AnalysisEngine().implementation()

    reader = _child_of_type(pipeline, CollectionReaderAnnotator)

    assert isinstance(reader.descriptor.parameters.camera_config, HsrRos2CameraConfig)


def test_detector_uses_the_model_and_threshold_of_the_engine(node):
    """
    The detector loads the configured model and drops detections below the configured
    confidence.
    """
    engine = AnalysisEngine(pretrained_model="custom.pt", detection_threshold=0.3)

    detector = _child_of_type(engine.implementation(), YoloAnnotator)

    assert detector.descriptor.parameters.pretrained_model == engine.pretrained_model
    assert detector.descriptor.parameters.threshold == engine.detection_threshold
