from __future__ import annotations

from collections.abc import Iterator

import numpy
import open3d
import pytest
from py_trees.common import Status

import robokudo.world as rk_world
from robokudo.annotators.exceptions import LocationNotInWorld
from robokudo.annotators.outputs import AnnotatorOutputPerPipelineMap
from robokudo.annotators.region_occupant_filter import RegionOccupantFilter
from robokudo.cas import CASViews
from robokudo.pipeline import Pipeline
from robokudo.types.camera import CameraObservation
from robokudo.types.scene import HumanHypothesis, ObjectHypothesis
from robokudo_msgs.action import Query
from robokudo_msgs.msg import ObjectDesignator
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Point2
from semantic_digital_twin.datastructures.camera_model import PinholeCameraModel
from semantic_digital_twin.datastructures.camera_resolution import CameraResolution
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.robot_parts import Camera
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    RegionSpecification,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Scale
from semantic_digital_twin.world_description.world_entity import Body, Region

# %% a world with one region in front of the camera


@pytest.fixture
def seat_region() -> Iterator[Region]:
    """
    A region two meters in front of a camera standing at the world origin, in the
    runtime world of RoboKudo.
    """
    world = World.create_with_root_body("map")
    region = RegionSpecification.box(
        "seat",
        Scale(1.0, 1.0, 1.0),
        parent_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(x=2.0),
    ).spawn(world)
    rk_world.set_world(world)
    yield region
    rk_world.clear_world()


def camera_at_the_origin_of(world: World) -> CameraObservation:
    """
    :return: An observation of a camera standing at the origin of ``world``, looking
        along its axes.
    """
    camera_body = Body(name=PrefixedName(name="camera"))
    camera_model = PinholeCameraModel(
        image_resolution=CameraResolution(width=200, height=200),
        focal_length_x=100.0,
        focal_length_y=100.0,
        principal_point=Point2(x=100.0, y=100.0),
    )
    return CameraObservation(
        camera=Camera(
            name=PrefixedName(name="camera"),
            root=camera_body,
            camera_model=camera_model,
        ),
        effective_camera_model=camera_model,
        world_T_camera=HomogeneousTransformationMatrix(
            reference_frame=world.root, child_frame=camera_body
        ),
        timestamp_nanoseconds=0,
    )


@pytest.fixture
def occupant_filter(
    seat_region: Region,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> RegionOccupantFilter:
    """
    A set up filter whose CAS sees the world from a camera at the world origin.
    """
    pipeline = Pipeline()
    annotator = RegionOccupantFilter()
    pipeline.add_child(annotator)
    pipeline.setup()
    pipeline.cas.camera_observation = camera_at_the_origin_of(seat_region._world)
    return annotator


def human_at(x: float) -> HumanHypothesis:
    """
    :return: A human hypothesis whose cloud is centered at ``x`` on the camera's x axis.
    """
    cloud = open3d.geometry.PointCloud()
    cloud.points = open3d.utility.Vector3dVector(
        numpy.array([[x - 0.1, 0.0, 0.0], [x + 0.1, 0.0, 0.0]])
    )
    return HumanHypothesis(points=cloud)


def query_for(location: str) -> Query.Goal:
    """
    :return: A query for the humans in ``location``.
    """
    return Query.Goal(obj=ObjectDesignator(type="human", location=location))


# %% keeping the humans in the queried region


def test_only_humans_inside_the_queried_region_remain(
    seat_region: Region, occupant_filter: RegionOccupantFilter
):
    inside, outside = human_at(2.0), human_at(0.0)
    cas = occupant_filter.get_cas()
    cas.annotations.extend([inside, outside])
    cas.set(CASViews.QUERY, query_for(seat_region.name.name))

    assert occupant_filter.update() == Status.SUCCESS

    assert cas.filter_annotations_by_type(HumanHypothesis) == [inside]


def test_other_hypotheses_are_left_alone(
    seat_region: Region, occupant_filter: RegionOccupantFilter
):
    unrelated = ObjectHypothesis()
    cas = occupant_filter.get_cas()
    cas.annotations.append(unrelated)
    cas.set(CASViews.QUERY, query_for(seat_region.name.name))

    occupant_filter.update()

    assert cas.annotations == [unrelated]


def test_a_query_naming_no_region_keeps_every_human(
    seat_region: Region, occupant_filter: RegionOccupantFilter
):
    humans = [human_at(2.0), human_at(0.0)]
    cas = occupant_filter.get_cas()
    cas.annotations.extend(humans)
    cas.set(CASViews.QUERY, query_for(""))

    occupant_filter.update()

    assert cas.filter_annotations_by_type(HumanHypothesis) == humans


def test_a_region_missing_from_the_world_is_rejected(
    seat_region: Region, occupant_filter: RegionOccupantFilter
):
    occupant_filter.get_cas().set(CASViews.QUERY, query_for("no_such_region"))

    with pytest.raises(LocationNotInWorld):
        occupant_filter.update()
