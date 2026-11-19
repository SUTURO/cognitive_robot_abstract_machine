"""
Tests of the analysis engine that answers queries with the objects YOLO detects in the
HSR head camera image and says them.
"""

# %% imports
from __future__ import annotations

import py_trees.composites
from py_trees.behaviour import Behaviour
from py_trees.common import Status

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
from robokudo.descriptors.analysis_engines.yolo_object_detection import (
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


def _behaviours_of_type(root: Behaviour, behaviour_type: type[Behaviour]) -> list:
    """
    :return: Every behaviour of the given type in the tree below the root.
    """
    return [
        behaviour
        for behaviour in root.iterate()
        if isinstance(behaviour, behaviour_type)
    ]


def _tick_to_completion(behaviour: Behaviour) -> Status:
    """
    :return: The status of the behaviour after ticking it until it stops running.
    """
    behaviour.tick_once()
    while behaviour.status == Status.RUNNING:
        behaviour.tick_once()
    return behaviour.status


# %% pipeline composition
def test_pipeline_answers_a_query_after_detecting_localizing_and_saying_objects(node):
    """
    After its initialization, the pipeline waits for a query, reads one image, detects,
    types and localizes the objects, says them and answers the query.
    """
    pipeline = AnalysisEngine().implementation()

    assert [type(child) for child in pipeline.children[1:]] == [
        QueryAnnotator,
        CollectionReaderAnnotator,
        ImagePreprocessorAnnotator,
        YoloAnnotator,
        SemanticTypeReplacementAnnotator,
        py_trees.composites.Selector,
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


# %% object localization
def test_empty_scene_succeeds_without_localizing(node):
    """
    Without detected objects, the localization is skipped and the query is still
    answered.
    """
    localization = AnalysisEngine().object_localization()
    pipeline = Pipeline("ObjectLocalization")
    pipeline.add_child(localization)
    pipeline.cas.annotations = []

    status = _tick_to_completion(localization)

    (outlier_removal,) = _behaviours_of_type(
        localization, OutlierRemovalOnObjectHypothesisAnnotator
    )
    assert status == Status.SUCCESS
    assert outlier_removal.status == Status.INVALID
