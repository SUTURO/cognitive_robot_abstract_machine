"""
Completed, unrestricted scene observations produce a single grounded report.
"""

from __future__ import annotations

from threading import Event, Thread
from unittest.mock import Mock

import pytest
from rclpy.publisher import Publisher
from std_msgs.msg import String

from coraplex.scene_perception import (
    ObservedObject,
    SceneObservation,
    ScenePerception,
    SceneQuery,
    SceneReport,
)
from krrood_spatial_math.spatial_types import Pose, HomogeneousTransformationMatrix
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
)
from semantic_digital_twin.world_description.geometry import VolumetricBoundingBox
from coraplex.exceptions import ScenePerceptionFailed
from semantic_digital_twin.semantic_annotations.semantic_annotations import Table
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Scale

# %% scene fixtures


@pytest.fixture
def scene_query() -> SceneQuery:
    """
    A named support whose body name does not encode its furniture category.
    """
    world = World.create_with_root_body("root")
    Table.create_with_new_body_in_world(
        world=world, name="arbitrary_support", scale=Scale(2, 2, 1)
    )
    return SceneQuery(world=world)


# %% completed batches


@pytest.mark.parametrize("count", [0, 1, 5])
def test_scene_reports_accept_unknown_object_counts(
    scene_query: SceneQuery, count: int
) -> None:
    """
    No expected cardinality is needed to finish a report.
    """
    objects = [
        ObservedObject(
            category="cup",
            pose=Pose.from_xyz_rpy(z=0.55, reference_frame=scene_query.world.root),
            dimensions=Scale(0.1, 0.1, 0.1),
            identity=str(index),
        )
        for index in range(count)
    ]
    source = Mock(spec=ScenePerception)
    source.observe.return_value = SceneObservation(objects=objects)
    publisher = Mock(spec=Publisher)
    SceneReport(source=source, publisher=publisher).announce(scene_query)
    assert publisher.publish.call_count == 1
    assert source.observe.call_args.args == (scene_query,)


def test_mixed_new_instances_are_grouped_without_world_entries(
    scene_query: SceneQuery,
) -> None:
    """
    Distinct same-category observations survive while duplicate identities collapse.
    """
    pose = Pose.from_xyz_rpy(z=0.55, reference_frame=scene_query.world.root)
    cup = ObservedObject(
        category="cup", pose=pose, dimensions=Scale(0.1, 0.1, 0.1), identity="first"
    )
    batch = SceneObservation(
        objects=[
            cup,
            cup,
            ObservedObject(
                category="cup",
                pose=pose,
                dimensions=Scale(0.1, 0.1, 0.1),
                identity="second",
            ),
            ObservedObject(
                category="bottle",
                pose=pose,
                dimensions=Scale(0.1, 0.1, 0.1),
                identity="third",
            ),
        ]
    )
    before = len(scene_query.world.bodies)
    assert batch.describe(scene_query) == "I detected 2 cups and a bottle on the table."
    assert len(scene_query.world.bodies) == before


def test_missing_geometry_does_not_invent_support(scene_query: SceneQuery) -> None:
    """
    A classification and pose alone are insufficient to establish contact.
    """
    batch = SceneObservation(
        objects=[
            ObservedObject(
                category="bowl", pose=Pose(reference_frame=scene_query.world.root)
            )
        ]
    )
    assert batch.describe(scene_query) == "I detected a bowl."


def test_unclassified_objects_remain_in_the_report(scene_query: SceneQuery) -> None:
    """
    An unknown category is reported honestly rather than discarded.
    """
    assert (
        SceneObservation(objects=[ObservedObject()]).describe(scene_query)
        == "I detected an unidentified object."
    )


def test_anonymous_same_category_objects_are_not_merged(
    scene_query: SceneQuery,
) -> None:
    """
    Missing identities do not justify assuming two hypotheses are duplicates.
    """
    assert (
        SceneObservation(
            objects=[ObservedObject(category="cup"), ObservedObject(category="cup")]
        ).describe(scene_query)
        == "I detected 2 cups."
    )


def test_speech_waits_for_the_completed_batch(scene_query: SceneQuery) -> None:
    """
    No report is published until the source finishes the request.
    """
    entered, release = Event(), Event()

    def observe(query: SceneQuery) -> SceneObservation:
        """
        Hold the result until the test completes the perception request.
        """
        entered.set()
        release.wait(timeout=5)
        return SceneObservation(objects=[ObservedObject(category="cup")])

    source = Mock(spec=ScenePerception)
    source.observe.side_effect = observe
    publisher = Mock(spec=Publisher)
    worker = Thread(
        target=SceneReport(source=source, publisher=publisher).announce,
        args=(scene_query,),
    )
    worker.start()
    try:
        assert entered.wait(timeout=5)
        publisher.publish.assert_not_called()
    finally:
        release.set()
        worker.join(timeout=5)
    assert not worker.is_alive()
    publisher.publish.assert_called_once_with(String(data="I detected a cup."))


def test_next_scan_does_not_reuse_previous_detections(scene_query: SceneQuery) -> None:
    """
    Each completed batch replaces the previous report's observation set.
    """
    source = Mock(spec=ScenePerception)
    source.observe.side_effect = [
        SceneObservation(objects=[ObservedObject(category="cup")]),
        SceneObservation(objects=[]),
    ]
    publisher = Mock(spec=Publisher)
    report = SceneReport(source=source, publisher=publisher)
    report.announce(scene_query)
    report.announce(scene_query)
    assert [call.args[0].data for call in publisher.publish.call_args_list] == [
        "I detected a cup.",
        "I did not detect any objects.",
    ]


# %% location evidence


def test_scene_region_excludes_outside_detections(scene_query: SceneQuery) -> None:
    """
    Objects localized outside the requested region do not enter its report.
    """
    scene_query.region = VolumetricBoundingBox(
        origin=HomogeneousTransformationMatrix(reference_frame=scene_query.world.root),
        min_x=-1,
        max_x=1,
        min_y=-1,
        max_y=1,
        min_z=-1,
        max_z=1,
    )
    batch = SceneObservation(
        objects=[
            ObservedObject(
                category="cup",
                pose=Pose.from_xyz_rpy(x=5, reference_frame=scene_query.world.root),
            )
        ]
    )
    assert batch.describe(scene_query) == "I did not detect any objects."


def test_support_uses_the_detection_reference_frame(scene_query: SceneQuery) -> None:
    """
    Camera-relative detections are transformed before testing furniture support.
    """
    camera = BodySpecification.box(
        name="camera",
        scale=Scale(0.01, 0.01, 0.01),
        parent_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(x=2),
    ).spawn(scene_query.world)
    observation = ObservedObject(
        category="cup",
        pose=Pose.from_xyz_rpy(x=-2, z=0.55, reference_frame=camera),
        dimensions=Scale(0.1, 0.1, 0.1),
    )
    assert (
        SceneObservation(objects=[observation]).describe(scene_query)
        == "I detected a cup on the table."
    )


def test_target_furniture_is_selected_by_reference(scene_query: SceneQuery) -> None:
    """
    The task supplies furniture identity rather than a fixed table name.
    """
    surface = scene_query.world.get_semantic_annotations_by_type(Table)[0]
    scene_query.furniture = surface
    assert (
        SceneObservation().describe(scene_query)
        == "I did not detect any objects on the table."
    )


def test_failed_batch_does_not_publish_an_empty_scene(scene_query: SceneQuery) -> None:
    """
    A failed source cannot turn into a successful empty observation.
    """
    source = Mock(spec=ScenePerception)
    source.observe.side_effect = ScenePerceptionFailed("test failure")
    publisher = Mock(spec=Publisher)
    with pytest.raises(ScenePerceptionFailed):
        SceneReport(source=source, publisher=publisher).announce(scene_query)
    publisher.publish.assert_not_called()
