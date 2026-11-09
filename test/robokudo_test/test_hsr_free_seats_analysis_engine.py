"""
Tests for the pipeline of the analysis engine reporting the people on a seat.
"""

from __future__ import annotations

import pytest

pytest.importorskip("ultralytics")

from robokudo.annotators.cluster_position import ClusterPositionAnnotator
from robokudo.annotators.human_and_pose import HumanAndPoseAnnotator
from robokudo.annotators.human_sitting import HumanSittingAnnotator
from robokudo.annotators.human_standing import HumanStandingAnnotator
from robokudo.annotators.query import GenerateQueryResult
from robokudo.annotators.region_occupant_filter import RegionOccupantFilter
from robokudo.descriptors.analysis_engines.hsr_free_seats import AnalysisEngine
from robokudo.descriptors.worlds.world_kitchen_lab import WorldDescriptor
from robokudo.pipeline import Pipeline
from semantic_digital_twin.predefined_maps.sofa_seats import SofaSeat
from semantic_digital_twin.semantic_annotations.semantic_annotations import Human
from test.robokudo_test.test_analysis_engine_query_composition import (
    bounded_build_time,
    install_fake_collection_reader_descriptor,
)

# %% pipeline


@pytest.fixture
def pipeline(monkeypatch) -> Pipeline:
    """
    The engine's pipeline, built without a camera.
    """
    install_fake_collection_reader_descriptor(monkeypatch)
    with bounded_build_time():
        return AnalysisEngine().implementation()


def test_humans_are_located_and_judged_then_restricted_to_the_region_before_replying(
    pipeline: Pipeline,
):
    """
    Humans are located and their posture judged before only the occupants of the region
    are kept and replied with.
    """
    expected_order = [
        HumanAndPoseAnnotator,
        ClusterPositionAnnotator,
        HumanStandingAnnotator,
        HumanSittingAnnotator,
        RegionOccupantFilter,
        GenerateQueryResult,
    ]

    assert [
        annotator_type
        for annotator_type in (type(node) for node in pipeline.children)
        if annotator_type in expected_order
    ] == expected_order


def test_people_are_reported_as_the_annotation_the_plan_asks_for(pipeline: Pipeline):
    [human_annotator] = [
        node for node in pipeline.children if isinstance(node, HumanAndPoseAnnotator)
    ]

    parameters = human_annotator.descriptor.parameters
    assert parameters.person_class_name == Human.__name__.lower()
    assert parameters.global_with_depth is True


# %% world


def test_the_kitchen_lab_world_holds_a_region_per_seat():
    world = WorldDescriptor().world

    assert all(
        world.is_kinematic_structure_entity_in_world_by_name(seat.value)
        for seat in SofaSeat
    )
