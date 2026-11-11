"""
Descriptions derived from observed bodies and their current support.
"""

from __future__ import annotations

from unittest.mock import Mock

from typing_extensions import TYPE_CHECKING

import pytest
from rclpy.publisher import Publisher
from std_msgs.msg import String

from coraplex.scene_description import (
    PerceptionAnnouncement,
    SceneAnnouncement,
    SceneDescription,
)
from coraplex.perception import Detection, PerceptionInterface, PerceptionQuery
from coraplex.exceptions import (
    AmbiguousDetection,
    PerceivedObjectNotInWorld,
    PerceptionSourceUnavailable,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Cup,
    Table,
    CounterTop,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Scale
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Pose

if TYPE_CHECKING:
    from cramph.context import ContextExtension
    from semantic_digital_twin.robots.pr2 import PR2

# %% scene descriptions


@pytest.mark.parametrize(
    "furniture_type, expected",
    [
        (Table, "There is a cup on the table."),
        (CounterTop, "There is a cup on the counter top."),
    ],
)
def test_description_resolves_the_supporting_furniture(
    furniture_type: type[Table] | type[CounterTop], expected: str
) -> None:
    """
    Furniture categories come from annotations rather than body names.
    """
    world = World.create_with_root_body("root")
    furniture_type.create_with_new_body_in_world(
        world=world, name="arbitrary_support", scale=Scale(1, 1, 1)
    )
    cup = Cup.create_with_new_body_in_world(
        world=world,
        name="arbitrary_object",
        scale=Scale(0.1, 0.1, 0.1),
        world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(z=0.55),
    )

    assert SceneDescription(world=world).describe([cup.root]) == expected


def test_unobserved_objects_are_not_announced() -> None:
    """
    A world model alone does not establish a current visual observation.
    """
    world = World.create_with_root_body("root")
    Cup.create_with_new_body_in_world(
        world=world, name="unseen", scale=Scale(0.1, 0.1, 0.1)
    )

    assert SceneDescription(world=world).describe([]) == "I do not see any objects."


def test_unsupported_object_does_not_get_a_furniture_location() -> None:
    """
    A visible object without a known support is described without guessing.
    """
    world = World.create_with_root_body("root")
    Table.create_with_new_body_in_world(
        world=world, name="support", scale=Scale(1, 1, 1)
    )
    cup = Cup.create_with_new_body_in_world(
        world=world,
        name="object",
        scale=Scale(0.1, 0.1, 0.1),
        world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(z=2),
    )

    assert (
        SceneDescription(world=world).describe([cup.root, cup.root]) == "I see a cup."
    )


# %% speech requests


def test_scene_announcement_publishes_one_description() -> None:
    """
    One explicit request publishes one complete scene description.
    """
    world = World.create_with_root_body("root")
    publisher = Mock(spec=Publisher)
    announcement = SceneAnnouncement(
        description=SceneDescription(world), publisher=publisher
    )

    announcement.announce([])

    publisher.publish.assert_called_once_with(String(data="I do not see any objects."))


# %% perception announcements


@pytest.mark.parametrize("repeat_detection", [False, True])
def test_perception_results_update_the_world_before_announcing(
    pr2_apartment_context: tuple[World, PR2, list[ContextExtension]],
    repeat_detection: bool,
) -> None:
    """
    Descriptions use the detected pose and announce duplicate results only once.
    """
    world, _, _ = pr2_apartment_context
    Table.create_with_new_body_in_world(
        world=world,
        name="report_support",
        scale=Scale(1, 1, 1),
        world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(x=10),
    )
    cup = Cup.create_with_new_body_in_world(
        world=world,
        name="report_object",
        scale=Scale(0.1, 0.1, 0.1),
        world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(x=10, z=2),
    )
    detection = Detection(
        semantic_annotation=Cup,
        pose=Pose.from_xyz_rpy(x=10, z=0.55, reference_frame=world.root),
    )
    source = Mock(spec=PerceptionInterface)
    source.look_for.return_value = (
        [detection, detection] if repeat_detection else [detection]
    )
    publisher = Mock(spec=Publisher)
    query = Mock(spec=PerceptionQuery, world=world, trust_detected_orientation=True)

    PerceptionAnnouncement(source=source, publisher=publisher).announce(query)

    source.look_for.assert_called_once_with(query)
    publisher.publish.assert_called_once_with(
        String(data="There is a cup on the table.")
    )
    assert cup.root.global_pose.position.z == detection.pose.position.z


def test_empty_perception_results_do_not_announce_world_objects(
    pr2_apartment_context: tuple[World, PR2, list[ContextExtension]],
) -> None:
    """
    Each announcement uses only the result of its own perception request.
    """
    world, _, _ = pr2_apartment_context
    Cup.create_with_new_body_in_world(
        world=world, name="unseen_report_object", scale=Scale(0.1, 0.1, 0.1)
    )
    source = Mock(spec=PerceptionInterface)
    source.look_for.return_value = []
    publisher = Mock(spec=Publisher)
    query = Mock(spec=PerceptionQuery, world=world, trust_detected_orientation=True)

    PerceptionAnnouncement(source=source, publisher=publisher).announce(query)

    publisher.publish.assert_called_once_with(String(data="I do not see any objects."))


@pytest.mark.parametrize(
    "object_count, exception_type",
    [(0, PerceivedObjectNotInWorld), (2, AmbiguousDetection)],
)
def test_unresolved_perception_results_are_not_announced(
    object_count: int,
    exception_type: type[PerceivedObjectNotInWorld] | type[AmbiguousDetection],
) -> None:
    """
    Unknown and ambiguous detections cannot produce a misleading spoken report.
    """
    world = World.create_with_root_body("root")
    for index in range(object_count):
        Cup.create_with_new_body_in_world(
            world=world, name=f"candidate_{index}", scale=Scale(0.1, 0.1, 0.1)
        )
    source = Mock(spec=PerceptionInterface)
    source.look_for.return_value = [
        Detection(semantic_annotation=Cup, pose=Pose(reference_frame=world.root))
    ]
    publisher = Mock(spec=Publisher)
    query = Mock(spec=PerceptionQuery, world=world, trust_detected_orientation=True)

    with pytest.raises(exception_type):
        PerceptionAnnouncement(source=source, publisher=publisher).announce(query)

    publisher.publish.assert_not_called()


def test_a_perception_failure_is_not_reported_as_an_empty_scene() -> None:
    """
    An unavailable backend raises its error instead of claiming no objects were seen.
    """
    source = Mock(spec=PerceptionInterface)
    source.look_for.side_effect = PerceptionSourceUnavailable(
        action_name="unavailable_perception"
    )
    publisher = Mock(spec=Publisher)
    query = Mock(spec=PerceptionQuery)

    with pytest.raises(PerceptionSourceUnavailable):
        PerceptionAnnouncement(source=source, publisher=publisher).announce(query)

    publisher.publish.assert_not_called()
