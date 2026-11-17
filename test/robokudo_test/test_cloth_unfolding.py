"""
Tests for perceiving a cloth while it is unfolded: segmenting it, finding its grasp
points, its fold and how straight it is stretched, and recording them on the T-shirt.
"""

from __future__ import annotations

import math

import cv2
import numpy as np
import pytest
from py_trees.blackboard import Blackboard
from py_trees.common import Status

import robokudo.pipeline
import robokudo.world
from krrood_spatial_math.spatial_types import Point3
from robokudo.annotators.cloth_fold_detection import ClothFoldAnnotator, FoldRegion
from robokudo.annotators.cloth_grasp_points import ClothGraspPointAnnotator
from robokudo.annotators.cloth_height_coloring import ClothHeightColoringAnnotator
from robokudo.annotators.cloth_outline import ClothOutlineAnnotator
from robokudo.annotators.cloth_state_connector import ClothStateConnector
from robokudo.annotators.cloth_stretch import ClothStretchAnnotator
from robokudo.annotators.core import BaseAnnotator
from robokudo.annotators.outputs import AnnotatorOutputs
from robokudo.annotators.exceptions import (
    ClothMaskMissing,
    TShirtNotUnique,
)
from robokudo.cas import CAS, CASViews
from robokudo.pipeline import Pipeline
from robokudo.types.annotation import Classification
from robokudo.types.camera import RegisteredDepthImage
from robokudo.types.cloth import (
    ClothFoldAnnotation,
    ClothGraspPointsAnnotation,
    ClothHypothesis,
    ClothStretchAnnotation,
)
from robokudo.types.cv import Point2D, Rect
from robokudo.types.scene import ObjectHypothesis
from semantic_digital_twin.semantic_annotations.semantic_annotations import TShirt
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.cloth import ClothFold, ClothGraspPoints

from .dataset.cloth_on_table import ClothOnTableScene

pytestmark = pytest.mark.usefixtures("annotator_output_buffer")

# %% fixtures


@pytest.fixture
def scene() -> ClothOnTableScene:
    """
    :return: A cloth on a table, seen from one metre above.
    """
    return ClothOnTableScene()


@pytest.fixture
def cas(scene: ClothOnTableScene) -> CAS:
    """
    :return: A CAS holding the scene's camera frame.
    """
    return scene.cas()


@pytest.fixture
def depth(cas: CAS) -> RegisteredDepthImage:
    """
    :return: The scene's depth image, registered with its camera.
    """
    return RegisteredDepthImage.from_cas(cas)


@pytest.fixture
def cloth(scene: ClothOnTableScene, cas: CAS) -> ClothHypothesis:
    """
    :return: The scene's cloth, as a hypothesis in the CAS.
    """
    hypothesis = ClothHypothesis(contour=scene.contour())
    cas.annotations.append(hypothesis)
    return hypothesis


def update_in_pipeline(annotator: BaseAnnotator, cas: CAS) -> Status:
    """
    :return: The status of updating the annotator in a pipeline holding the CAS.
    """
    pipeline = Pipeline()
    pipeline.add_child(annotator)
    pipeline.cas = cas
    Blackboard().get("annotator_output_pipeline_map_buffer").map[pipeline.name] = (
        AnnotatorOutputs()
    )
    return annotator.update()


def grasp_points_annotation(
    left_pixel: Point2D, right_pixel: Point2D
) -> ClothGraspPointsAnnotation:
    """
    :return: Grasp points at the given pixels, at the world origin.
    """
    return ClothGraspPointsAnnotation(
        left_pixel=left_pixel,
        right_pixel=right_pixel,
        grasp_points=ClothGraspPoints(
            left_grasp_point=Point3(0.0, 0.0, 0.0),
            right_grasp_point=Point3(0.0, 0.0, 0.0),
        ),
    )


# %% outlining the detected cloth


def add_segmented_detection(
    cas: CAS, scene: ClothOnTableScene, classname: str, confidence: float
) -> ObjectHypothesis:
    """
    Add an object hypothesis covering the scene's cloth, segmented and classified as the
    open-vocabulary detector would.
    """
    hypothesis = ObjectHypothesis()
    hypothesis.roi.roi = scene.cloth_region
    hypothesis.roi.mask = np.full(
        (scene.cloth_region.height, scene.cloth_region.width),
        ClothOutlineAnnotator.mask_foreground,
        dtype=np.uint8,
    )
    hypothesis.annotations.append(
        Classification(classname=classname, confidence=confidence)
    )
    cas.annotations.append(hypothesis)
    return hypothesis


def cloth_class_name() -> str:
    """
    :return: The first class name the outline annotator takes for a cloth.
    """
    return ClothOutlineAnnotator().descriptor.parameters.cloth_class_names[0]


def test_the_detected_cloth_is_outlined_by_its_mask(
    scene: ClothOnTableScene, cas: CAS
) -> None:
    """
    The outline of the detected cloth's mask lies where the cloth is in the image.
    """
    detection = add_segmented_detection(cas, scene, cloth_class_name(), 0.8)

    update_in_pipeline(ClothOutlineAnnotator(), cas)

    (cloth,) = cas.filter_annotations_by_type(ClothHypothesis)
    left, top, width, height = cv2.boundingRect(cloth.contour)
    assert Rect.from_corner_points(left, top, left + width, top + height) == (
        scene.cloth_region
    )
    assert cloth.roi is detection.roi


def test_the_most_confidently_detected_cloth_is_outlined(
    scene: ClothOnTableScene, cas: CAS
) -> None:
    """
    Of several detected cloths, only the most confident one becomes the cloth.
    """
    add_segmented_detection(cas, scene, cloth_class_name(), 0.3)
    add_segmented_detection(cas, scene, cloth_class_name(), 0.8)

    update_in_pipeline(ClothOutlineAnnotator(), cas)

    (cloth,) = cas.filter_annotations_by_type(ClothHypothesis)
    (classification,) = CAS.filter_by_type(Classification, cloth.annotations)
    assert classification.confidence == 0.8


def test_a_detection_of_something_else_is_no_cloth(
    scene: ClothOnTableScene, cas: CAS
) -> None:
    """
    An object classified as something other than a cloth is not outlined.
    """
    add_segmented_detection(cas, scene, "sock", 0.8)

    update_in_pipeline(ClothOutlineAnnotator(), cas)

    assert cas.filter_annotations_by_type(ClothHypothesis) == []


def test_a_cloth_needs_a_mask_to_be_outlined(
    scene: ClothOnTableScene, cas: CAS
) -> None:
    """
    A cloth detected without being segmented cannot be outlined.
    """
    detection = add_segmented_detection(cas, scene, cloth_class_name(), 0.8)
    detection.roi.mask = None

    with pytest.raises(ClothMaskMissing):
        update_in_pipeline(ClothOutlineAnnotator(), cas)


# %% grasp points


def upper_edge_grasp_point_annotator() -> ClothGraspPointAnnotator:
    """
    :return: A grasp point search in the upper fifth of the cloth.
    """
    descriptor = ClothGraspPointAnnotator.Descriptor()
    descriptor.parameters.examined_height_fraction = 0.2
    return ClothGraspPointAnnotator(descriptor=descriptor)


def test_the_grasp_points_are_the_upper_corners_moved_into_the_cloth(
    scene: ClothOnTableScene,
) -> None:
    """
    Each upper corner of the rectangular cloth is moved a tenth of the way towards the
    middle of its upper edge.
    """
    grasp_pixels = upper_edge_grasp_point_annotator().grasp_pixels(scene.contour())

    assert grasp_pixels.left_pixel == Point2D(x=113, y=110)
    assert grasp_pixels.right_pixel == Point2D(x=185, y=110)


def test_the_grasp_points_are_annotated_in_the_world(
    cas: CAS, cloth: ClothHypothesis, depth: RegisteredDepthImage
) -> None:
    """
    The cloth is annotated with the world points the camera saw at the grasp pixels.
    """
    annotator = upper_edge_grasp_point_annotator()

    update_in_pipeline(annotator, cas)

    (annotation,) = CAS.filter_by_type(ClothGraspPointsAnnotation, cloth.annotations)
    expected_pixels = annotator.grasp_pixels(cloth.contour)
    assert annotation.left_pixel == expected_pixels.left_pixel
    assert annotation.right_pixel == expected_pixels.right_pixel
    assert np.allclose(
        annotation.grasp_points.left_grasp_point.to_np(),
        depth.world_point(expected_pixels.left_pixel).to_np(),
    )
    assert np.allclose(
        annotation.grasp_points.right_grasp_point.to_np(),
        depth.world_point(expected_pixels.right_pixel).to_np(),
    )


def test_no_grasp_points_are_annotated_without_depth_at_them(
    cas: CAS, cloth: ClothHypothesis
) -> None:
    """
    A grasp point the camera measured no depth at cannot be located in the world.
    """
    cas.depth_image[:] = 0

    update_in_pipeline(upper_edge_grasp_point_annotator(), cas)

    assert CAS.filter_by_type(ClothGraspPointsAnnotation, cloth.annotations) == []


# %% height coloring


def test_the_cloth_is_colored_from_green_at_its_lowest_to_blue_at_its_highest() -> None:
    """
    The cloth's lowest pixel turns green, its highest blue, and the rest of the image
    stays as it is.
    """
    image = np.full((1, 3, 3), 7, dtype=np.uint8)

    ClothHeightColoringAnnotator.color_by_height(image, np.array([[0.0, 0.5, np.nan]]))

    assert image[0, 0].tolist() == [0, 255, 0]
    assert image[0, 1].tolist() == [255, 0, 0]
    assert image[0, 2].tolist() == [7, 7, 7]


def test_the_heights_of_the_cloth_are_measured_in_the_world(
    cas: CAS, cloth: ClothHypothesis, depth: RegisteredDepthImage
) -> None:
    """
    Each pixel of the cloth holds the world height the camera saw there, the table
    around it none.
    """
    update_in_pipeline(ClothHeightColoringAnnotator(), cas)

    pixel_on_cloth = Point2D(x=150, y=150)
    assert cloth.heights[pixel_on_cloth.y, pixel_on_cloth.x] == pytest.approx(
        depth.world_point(pixel_on_cloth).to_np()[2]
    )
    assert np.isnan(cloth.heights[0, 0])


def test_the_height_colored_image_is_stored_in_the_cas(
    scene: ClothOnTableScene, cas: CAS, cloth: ClothHypothesis
) -> None:
    """
    The image the fold detection model looks at shows the cloth colored by height and
    the table as the camera saw it.
    """
    update_in_pipeline(ClothHeightColoringAnnotator(), cas)

    height_colored_image = cas.get(CASViews.HEIGHT_COLORED_IMAGE)
    expected_image = scene.color_image()
    ClothHeightColoringAnnotator.color_by_height(expected_image, cloth.heights)
    assert (height_colored_image == expected_image).all()
    assert (height_colored_image[0, 0] == scene.color_image()[0, 0]).all()


# %% fold detection


@pytest.fixture
def measured_cloth(cas: CAS, cloth: ClothHypothesis) -> ClothHypothesis:
    """
    :return: The scene's cloth, with its heights measured.
    """
    update_in_pipeline(ClothHeightColoringAnnotator(), cas)
    return cloth


@pytest.fixture
def fold_region() -> FoldRegion:
    """
    :return: A fold region around the pixel in the middle of the scene's cloth.
    """
    return FoldRegion(
        region=Rect.from_corner_points(140, 140, 160, 160), confidence=0.9
    )


def add_detection(cas: CAS, fold_region: FoldRegion, classname: str) -> None:
    """
    Add an object hypothesis covering the region, classified as the detector would.
    """
    hypothesis = ObjectHypothesis()
    hypothesis.roi.roi = fold_region.region
    hypothesis.annotations.append(
        Classification(classname=classname, confidence=fold_region.confidence)
    )
    cas.annotations.append(hypothesis)


def fold_class_name() -> str:
    """
    :return: The class name the fold annotator takes for a fold.
    """
    return ClothFoldAnnotator().descriptor.parameters.fold_class_name


def test_a_fold_opens_towards_where_the_cloth_is_highest(
    measured_cloth: ClothHypothesis, fold_region: FoldRegion
) -> None:
    """
    The scene's cloth rises towards the right of the image, so the fold opens there.
    """
    direction = ClothFoldAnnotator().opening_direction(
        fold_region, measured_cloth.heights
    )

    assert direction == 0.0


@pytest.mark.parametrize(
    ("direction", "expected_edge_pixel"),
    [(0.0, Point2D(x=189, y=150)), (math.pi, Point2D(x=110, y=150))],
)
def test_the_edge_is_where_the_walk_reaches_the_outline(
    scene: ClothOnTableScene, direction: float, expected_edge_pixel: Point2D
) -> None:
    """
    Walking from the middle of the cloth, the edge is the first pixel on its outline.
    """
    edge_pixel = ClothFoldAnnotator.edge_pixel(
        Point2D(x=150, y=150),
        direction,
        scene.contour(),
        (scene.image_size, scene.image_size),
    )

    assert edge_pixel == expected_edge_pixel


def test_the_fold_is_laid_as_far_beyond_the_edge_as_the_edge_is_from_it(
    cas: CAS,
    measured_cloth: ClothHypothesis,
    depth: RegisteredDepthImage,
    fold_region: FoldRegion,
) -> None:
    """
    The fold is taken at the middle of its detected region and laid down mirrored at the
    edge of the cloth in the direction it opens towards.
    """
    add_detection(cas, fold_region, fold_class_name())

    update_in_pipeline(ClothFoldAnnotator(), cas)

    (annotation,) = CAS.filter_by_type(ClothFoldAnnotation, measured_cloth.annotations)
    fold_point = depth.world_point(fold_region.center)
    edge_point = depth.world_point(annotation.edge_pixel)
    assert annotation.edge_pixel == Point2D(x=189, y=150)
    assert annotation.fold_region == fold_region.region
    assert np.allclose(annotation.fold.fold_point.to_np(), fold_point.to_np())
    assert np.allclose(
        annotation.fold.unfold_target.to_np()[:3],
        2 * edge_point.to_np()[:3] - fold_point.to_np()[:3],
    )


def test_the_most_confident_fold_on_the_cloth_is_located(
    cas: CAS, measured_cloth: ClothHypothesis, fold_region: FoldRegion
) -> None:
    """
    Of several folds detected on the cloth, the most confident one is located.
    """
    less_confident = FoldRegion(
        region=Rect.from_corner_points(120, 120, 130, 130), confidence=0.5
    )
    add_detection(cas, less_confident, fold_class_name())
    add_detection(cas, fold_region, fold_class_name())

    update_in_pipeline(ClothFoldAnnotator(), cas)

    (annotation,) = CAS.filter_by_type(ClothFoldAnnotation, measured_cloth.annotations)
    assert annotation.confidence == fold_region.confidence


@pytest.mark.parametrize(
    ("detected_region", "classname"),
    [
        (Rect.from_corner_points(140, 140, 160, 160), "sock"),
        (Rect.from_corner_points(10, 10, 30, 30), None),
    ],
    ids=["another class", "off the cloth"],
)
def test_no_fold_is_located_without_a_fold_detected_on_the_cloth(
    cas: CAS,
    measured_cloth: ClothHypothesis,
    detected_region: Rect,
    classname: str | None,
) -> None:
    """
    A detection of something else, or of a fold beside the cloth, is no fold in it.
    """
    add_detection(
        cas,
        FoldRegion(region=detected_region, confidence=0.9),
        classname if classname is not None else fold_class_name(),
    )

    update_in_pipeline(ClothFoldAnnotator(), cas)

    assert CAS.filter_by_type(ClothFoldAnnotation, measured_cloth.annotations) == []


def test_no_fold_is_located_on_a_cloth_whose_heights_are_unmeasured(
    cas: CAS, cloth: ClothHypothesis, fold_region: FoldRegion
) -> None:
    """
    Without the cloth's heights, the direction the fold opens in is unknown.
    """
    add_detection(cas, fold_region, fold_class_name())

    update_in_pipeline(ClothFoldAnnotator(), cas)

    assert CAS.filter_by_type(ClothFoldAnnotation, cloth.annotations) == []


# %% stretching


def stretch_annotator() -> ClothStretchAnnotator:
    """
    :return: A stretch check that leaves ten pixels out next to each gripper.
    """
    descriptor = ClothStretchAnnotator.Descriptor()
    descriptor.parameters.gripper_clearance = 10
    return ClothStretchAnnotator(descriptor=descriptor)


def test_a_cloth_with_a_straight_upper_edge_is_stretched(cas: CAS) -> None:
    """
    The edge between grippers at the upper corners of the rectangle is straight.
    """
    cloth = ClothHypothesis(contour=ClothOnTableScene().contour())
    cloth.annotations.append(
        grasp_points_annotation(Point2D(x=113, y=110), Point2D(x=185, y=110))
    )
    cas.annotations.append(cloth)

    update_in_pipeline(stretch_annotator(), cas)

    (annotation,) = CAS.filter_by_type(ClothStretchAnnotation, cloth.annotations)
    assert annotation.maximum_deviation == 0.0
    assert annotation.is_stretched


def test_a_sagging_cloth_is_not_stretched(cas: CAS) -> None:
    """
    An upper edge that sags twenty pixels between the grippers is not straight.
    """
    sagging_outline = np.array(
        [[[110, 110]], [[110, 189]], [[189, 189]], [[189, 110]], [[150, 130]]],
        dtype=np.int32,
    )
    cloth = ClothHypothesis(contour=sagging_outline)
    cloth.annotations.append(
        grasp_points_annotation(Point2D(x=113, y=110), Point2D(x=185, y=110))
    )
    cas.annotations.append(cloth)

    update_in_pipeline(stretch_annotator(), cas)

    (annotation,) = CAS.filter_by_type(ClothStretchAnnotation, cloth.annotations)
    assert annotation.maximum_deviation == 20.0
    assert not annotation.is_stretched


def test_a_cloth_without_grasp_points_is_not_checked_for_stretching(
    cas: CAS, cloth: ClothHypothesis
) -> None:
    """
    Without knowing where the grippers hold the cloth, there is no edge between them.
    """
    update_in_pipeline(stretch_annotator(), cas)

    assert CAS.filter_by_type(ClothStretchAnnotation, cloth.annotations) == []


# %% recording on the T-shirt


@pytest.fixture
def t_shirt() -> TShirt:
    """
    :return: A T-shirt in RoboKudo's world.
    """
    world = World.create_with_root_body("root")
    robokudo.world.set_world(world)
    with world.modify_world():
        return TShirt.create_with_new_body_in_world(name="t_shirt", world=world)


@pytest.fixture
def perceived_fold(t_shirt: TShirt) -> ClothFold:
    """
    :return: A fold perception found in the T-shirt.
    """
    root = t_shirt._world.root
    return ClothFold(
        fold_point=Point3(0.1, 0.0, 0.02, reference_frame=root),
        unfold_target=Point3(0.3, 0.0, 0.0, reference_frame=root),
    )


def test_the_t_shirt_records_what_perception_found(
    cas: CAS, cloth: ClothHypothesis, t_shirt: TShirt, perceived_fold: ClothFold
) -> None:
    """
    The T-shirt takes the grasp points and the fold the cloth is annotated with.
    """
    grasp_points = grasp_points_annotation(Point2D(x=0, y=0), Point2D(x=1, y=0))
    cloth.annotations.append(grasp_points)
    cloth.annotations.append(
        ClothFoldAnnotation(
            fold_region=Rect(),
            confidence=0.9,
            edge_pixel=Point2D(),
            fold=perceived_fold,
        )
    )

    update_in_pipeline(ClothStateConnector(), cas)

    assert t_shirt.grasp_points is grasp_points.grasp_points
    assert t_shirt.fold is perceived_fold


def test_the_t_shirt_forgets_a_fold_perception_no_longer_finds(
    cas: CAS, cloth: ClothHypothesis, t_shirt: TShirt, perceived_fold: ClothFold
) -> None:
    """
    A cloth annotated without a fold clears the fold recorded before.
    """
    with t_shirt._world.modify_world():
        t_shirt.update_fold(perceived_fold)

    update_in_pipeline(ClothStateConnector(), cas)

    assert t_shirt.fold is None


def test_perception_of_a_cloth_needs_exactly_one_t_shirt_in_the_world(
    cas: CAS, cloth: ClothHypothesis
) -> None:
    """
    Without a T-shirt in the world, it is unclear what the cloth is.
    """
    with pytest.raises(TShirtNotUnique):
        update_in_pipeline(ClothStateConnector(), cas)


def test_nothing_is_recorded_without_a_cloth(
    cas: CAS, t_shirt: TShirt, perceived_fold: ClothFold
) -> None:
    """
    A frame without a cloth leaves the T-shirt as it was.
    """
    with t_shirt._world.modify_world():
        t_shirt.update_fold(perceived_fold)

    update_in_pipeline(ClothStateConnector(), cas)

    assert t_shirt.fold is perceived_fold
