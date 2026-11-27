"""
Tests for the human detection analysis engine.
"""

from __future__ import annotations

# %% imports
from unittest.mock import Mock, call

import pytest
from rclpy.validate_topic_name import validate_topic_name
from typing_extensions import TYPE_CHECKING

from robokudo.annotators import human_and_pose
from robokudo.annotators.gpsr.sitting import HumanPosture
from robokudo.annotators.human_and_pose import HumanAndPoseAnnotator
from robokudo.descriptors.analysis_engines.human_detection import (
    AnalysisEngine,
    HumanRoute,
)

from ._gpsr_routing import gpsr_query, scheduler_of

if TYPE_CHECKING:
    from pathlib import Path
    from rclpy.node import Node


# %% model setup
def test_pose_model_setup_uses_the_configured_weights(
    node: Node, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """
    Configure the model loader without loading or downloading weights.
    """
    pipeline = AnalysisEngine().implementation()
    annotator = next(
        child
        for child in pipeline.iterate()
        if isinstance(child, HumanAndPoseAnnotator)
    )
    annotator.descriptor.parameters.models_directory = tmp_path
    annotator.descriptor.parameters.warmup_annotator = False
    model_loader = Mock()
    monkeypatch.setattr(human_and_pose, "YOLO", model_loader)

    annotator.setup()

    assert model_loader.call_args_list == [
        call(str(tmp_path / annotator.descriptor.parameters.pretrained_model))
    ]


# %% query routing
@pytest.mark.parametrize(
    ("attributes", "expected_route"),
    [
        ([], HumanRoute.DETECTION),
        ([HumanPosture.STANDING], HumanRoute.POSTURE),
        ([HumanPosture.SITTING], HumanRoute.POSTURE),
        ([HumanPosture.LAYING], HumanRoute.POSTURE),
        (["waving"], None),
        (["standing", "sitting"], None),
    ],
)
def test_human_queries_select_only_supported_routes(
    node: Node, attributes: list[str], expected_route: HumanRoute | None
) -> None:
    """
    Choose a route only when the requested human query is supported.
    """
    scheduler = scheduler_of(AnalysisEngine().implementation())
    assert scheduler.filter_fn(gpsr_query("human", attributes)) == expected_route


def test_other_types_have_no_human_route(node: Node) -> None:
    """
    Reject queries that do not request humans.
    """
    scheduler = scheduler_of(AnalysisEngine().implementation())
    assert scheduler.filter_fn(gpsr_query("object")) is None


# %% pipeline structure
def test_pipeline_name_is_a_valid_visualization_topic(node: Node) -> None:
    """
    The pipeline can expose its visualizer through ROS.
    """
    pipeline = AnalysisEngine().implementation()
    validate_topic_name(f"{pipeline.name}/output_image")


def test_scheduler_starts_its_own_sequence(node: Node) -> None:
    """
    The scheduler is the first child of the sequence it manages.
    """
    scheduler = scheduler_of(AnalysisEngine().implementation())
    assert scheduler.parent.children[0] is scheduler
