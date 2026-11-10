"""
Tests for the seat places of the shared world as seats of the free seat detection.
"""

# %% imports

from __future__ import annotations

import pytest
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Point2
from semantic_digital_twin.datastructures.camera_model import PinholeCameraModel
from semantic_digital_twin.datastructures.camera_resolution import CameraResolution
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.robot_parts import Camera
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Chair,
    HasSeatPlaces,
    Sofa,
)
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
    RegionSpecification,
)
from semantic_digital_twin.specifications.semantic_annotations import (
    SemanticAnnotationWithRootSpecification,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Scale
from semantic_digital_twin.world_description.world_entity import Body, Region

from robokudo.annotators.gpsr.sitting import HumanPosture
from robokudo.annotators.seat_detection import SeatFurniture, WorldSeatRegion
from robokudo.annotators.seat_occupancy import SeatOccupancyAnnotator
from robokudo.annotators.world_seat_regions import WorldSeatRegionAnnotator
from robokudo.cas import CASViews
from robokudo.pipeline import Pipeline
from robokudo.types.camera import CameraObservation
from robokudo.types.cv import ColorToDepthRatio
from .._marked_image import MarkedRegionImage
from ._seating_scene import (
    annotate,
    human,
    located,
    occupancies,
    seat,
    seat_region,
)
from .test_seat_query_answer import (
    add_occupancy,
    answer_query,
    camera_observation,
    camera_translation,
    world_frame_name,
)

# %% a world with seat places in front of a camera


def furniture_with_one_seat_place(
    world: World,
    furniture_type: type[HasSeatPlaces],
    name: str,
    camera_T_seat_place: HomogeneousTransformationMatrix,
) -> Region:
    """
    Spawn a piece of furniture with one seat place, as seen from a camera standing at
    the origin of ``world`` and looking along its z axis.

    :return: The seat place.
    """
    seat_place_name = f"{name}_seat_place"
    furniture = SemanticAnnotationWithRootSpecification(
        name=name,
        semantic_annotation_type=furniture_type,
        root_specification=BodySpecification(
            name=name,
            child_specifications=[
                RegionSpecification.box(seat_place_name, Scale(0.5, 0.5, 0.5))
            ],
        ),
    ).spawn(world, parent_T_self=camera_T_seat_place)
    seat_place = world.get_kinematic_structure_entity_by_name(seat_place_name)
    furniture.seat_places.append(seat_place)
    return seat_place


@pytest.fixture
def world() -> World:
    """
    An empty world whose origin the camera stands at.
    """
    return World.create_with_root_body("map")


@pytest.fixture
def camera_model() -> PinholeCameraModel:
    """
    The calibration of the camera, matching the image of :class:`MarkedRegionImage`.
    """
    image = MarkedRegionImage()
    return PinholeCameraModel(
        image_resolution=CameraResolution(width=image.width, height=image.height),
        focal_length_x=500.0,
        focal_length_y=500.0,
        principal_point=Point2(x=image.width / 2, y=image.height / 2),
    )


def regions_seen_by(
    world: World, camera_model: PinholeCameraModel
) -> list[WorldSeatRegion]:
    """
    Run a :class:`WorldSeatRegionAnnotator` on ``world`` seen by a camera at its origin.

    :return: The world seat regions it added to the CAS.
    """
    annotator = WorldSeatRegionAnnotator()
    annotator.world = world
    pipeline = Pipeline()
    pipeline.add_child(annotator)
    pipeline.setup()
    camera_body = Body(name=PrefixedName(name="camera"))
    pipeline.cas.camera_observation = CameraObservation(
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
    pipeline.cas.pointcloud_camera_model = camera_model
    pipeline.cas.color_to_depth_ratio = ColorToDepthRatio()
    pipeline.cas.set(CASViews.COLOR_IMAGE, MarkedRegionImage().color_image())
    annotator.update()
    return pipeline.cas.filter_annotations_by_type(WorldSeatRegion)


# %% seat places as seats


def test_a_seat_place_in_view_becomes_a_seat_region_named_after_it(
    world: World, camera_model: PinholeCameraModel
):
    """
    A seat place in front of the camera is added as a seat region carrying its name.
    """
    seat_place = furniture_with_one_seat_place(
        world, Chair, "chair", HomogeneousTransformationMatrix.from_xyz_rpy(z=2.0)
    )

    [seat_region_of_place] = regions_seen_by(world, camera_model)

    assert seat_region_of_place.seat_place_name == seat_place.name.name


def test_a_seat_region_is_positioned_at_the_center_of_its_seat_place(
    world: World, camera_model: PinholeCameraModel
):
    """
    The seat region lies where the center of its seat place is in the camera frame.
    """
    furniture_with_one_seat_place(
        world,
        Chair,
        "chair",
        HomogeneousTransformationMatrix.from_xyz_rpy(x=0.3, y=-0.2, z=2.0),
    )

    [seat_region_of_place] = regions_seen_by(world, camera_model)

    [position] = seat_region_of_place.annotations
    assert position.translation == pytest.approx([0.3, -0.2, 2.0])


def test_a_seat_region_frames_its_seat_place_in_the_image(
    world: World, camera_model: PinholeCameraModel
):
    """
    The image region of a seat place straight ahead is centered in the image.
    """
    furniture_with_one_seat_place(
        world, Chair, "chair", HomogeneousTransformationMatrix.from_xyz_rpy(z=2.0)
    )

    [seat_region_of_place] = regions_seen_by(world, camera_model)

    left, top, right, bottom = seat_region_of_place.roi.roi.get_corner_points()
    image = MarkedRegionImage()
    assert (left + right) / 2 == pytest.approx(image.width / 2, abs=1)
    assert (top + bottom) / 2 == pytest.approx(image.height / 2, abs=1)


def test_a_seat_place_behind_the_camera_is_left_out(
    world: World, camera_model: PinholeCameraModel
):
    """
    A seat place behind the camera is not seen and so not added.
    """
    furniture_with_one_seat_place(
        world, Chair, "chair", HomogeneousTransformationMatrix.from_xyz_rpy(z=-2.0)
    )

    assert regions_seen_by(world, camera_model) == []


@pytest.mark.parametrize(
    "furniture_type, furniture",
    [(Chair, SeatFurniture.CHAIR), (Sofa, SeatFurniture.SOFA)],
)
def test_a_seat_region_names_the_furniture_of_its_seat_place(
    world: World,
    camera_model: PinholeCameraModel,
    furniture_type: type[HasSeatPlaces],
    furniture: SeatFurniture,
):
    """
    The seat region of a seat place says what kind of furniture the place belongs to.
    """
    furniture_with_one_seat_place(
        world,
        furniture_type,
        "furniture",
        HomogeneousTransformationMatrix.from_xyz_rpy(z=2.0),
    )

    [seat_region_of_place] = regions_seen_by(world, camera_model)

    assert seat_region_of_place.furniture == furniture


# %% world seat regions among the detected seats


def world_seat_region(x: float, z: float) -> WorldSeatRegion:
    """
    :return: A world seat region of the seat place ``seat_place`` at the given position.
    """
    region = WorldSeatRegion(seat_place_name="seat_place")
    region.roi.roi = MarkedRegionImage().region
    located(region, x=x, z=z)
    return region


def test_a_chair_detected_on_a_world_seat_region_is_left_out(
    parameters: SeatOccupancyAnnotator.Descriptor.Parameters, seat_distance: float
) -> None:
    """
    A chair detected closer to a world seat region than the minimum seat spacing is the
    same place, and only the world seat region is annotated.
    """
    place = world_seat_region(x=0.0, z=seat_distance)
    chair = located(seat(), x=parameters.minimum_seat_spacing / 2, z=seat_distance)

    annotate(SeatOccupancyAnnotator(), place, chair)

    assert (occupancies(place), occupancies(chair)) == ([False], [])


def test_a_chair_detected_apart_from_a_world_seat_region_is_annotated_too(
    parameters: SeatOccupancyAnnotator.Descriptor.Parameters, seat_distance: float
) -> None:
    """
    A chair detected farther from a world seat region than the minimum seat spacing is
    another seat.
    """
    place = world_seat_region(x=0.0, z=seat_distance)
    chair = located(seat(), x=parameters.minimum_seat_spacing * 2, z=seat_distance)

    annotate(SeatOccupancyAnnotator(), place, chair)

    assert (occupancies(place), occupancies(chair)) == ([False], [False])


def test_a_split_sofa_seat_on_a_world_seat_region_is_left_out(
    parameters: SeatOccupancyAnnotator.Descriptor.Parameters, seat_distance: float
) -> None:
    """
    A seat region split from a detected sofa on a world seat region is the same place.
    """
    place = world_seat_region(x=0.0, z=seat_distance)
    split_seat = seat_region(x=parameters.minimum_seat_spacing / 2, z=seat_distance)

    annotate(SeatOccupancyAnnotator(), place, split_seat)

    assert (occupancies(place), occupancies(split_seat)) == ([False], [])


def test_a_sitting_human_occupies_a_world_seat_region(seat_distance: float) -> None:
    """
    A sitting human at a world seat region occupies it.
    """
    place = world_seat_region(x=0.0, z=seat_distance)
    person = located(human(HumanPosture.SITTING), x=0.0, z=seat_distance)

    annotate(SeatOccupancyAnnotator(), place, person)

    assert occupancies(place) == [True]


# %% answering with world seat regions


def test_a_world_seat_region_is_reported_with_the_name_of_its_seat_place(
    camera_observation: CameraObservation, seat_distance: float
) -> None:
    """
    A world seat region is reported identified by its seat place, a chair without one.
    """
    place = add_occupancy(world_seat_region(x=-0.5, z=seat_distance), occupied=True)
    chair = add_occupancy(located(seat(), x=0.5, z=seat_distance), occupied=False)

    answer = answer_query(camera_observation, place, chair)

    assert [designator.uid for designator in answer.res] == [
        place.seat_place_name,
        "",
    ]
