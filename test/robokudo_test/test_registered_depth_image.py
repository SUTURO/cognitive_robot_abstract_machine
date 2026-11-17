"""
Tests for looking up color image pixels as the world points the camera saw there.
"""

from __future__ import annotations

import numpy as np
import pytest

from robokudo.types.camera import RegisteredDepthImage
from robokudo.types.cv import ColorToDepthRatio, Point2D

from .dataset.cloth_on_table import ClothOnTableScene

# %% fixtures


@pytest.fixture
def scene() -> ClothOnTableScene:
    """
    :return: A cloth on a table, seen from one metre above.
    """
    return ClothOnTableScene()


@pytest.fixture
def depth(scene: ClothOnTableScene) -> RegisteredDepthImage:
    """
    :return: The scene's depth image, registered with its camera.
    """
    return RegisteredDepthImage.from_cas(scene.cas())


# %% world points


def test_the_pixel_on_the_optical_axis_sees_the_table_below_the_camera(
    depth: RegisteredDepthImage,
) -> None:
    """
    The camera looks straight down onto the table at the world origin.
    """
    point = depth.world_point(Point2D(x=100, y=100))

    assert np.allclose(point.to_np()[:3], [0.0, 0.0, 0.0])


def test_a_pixel_right_of_the_optical_axis_sees_along_the_world_x_axis(
    depth: RegisteredDepthImage,
) -> None:
    """
    Fifty pixels at a focal length of a hundred pixels and a depth of one metre are half
    a metre.
    """
    point = depth.world_point(Point2D(x=150, y=100))

    assert np.allclose(point.to_np()[:3], [0.5, 0.0, 0.0])


def test_a_pixel_below_the_optical_axis_sees_against_the_world_y_axis(
    depth: RegisteredDepthImage,
) -> None:
    """
    The camera is rolled over to look down, so the image's downward direction points
    against the world's y-axis.
    """
    point = depth.world_point(Point2D(x=100, y=150))

    assert np.allclose(point.to_np()[:3], [0.0, -0.5, 0.0])


def test_a_point_is_given_in_the_frame_of_the_camera_pose(
    scene: ClothOnTableScene, depth: RegisteredDepthImage
) -> None:
    """
    The point is in the world the camera's pose is given in.
    """
    point = depth.world_point(Point2D(x=100, y=100))

    assert point.reference_frame is scene.world_frame


def test_a_pixel_without_depth_sees_no_point(
    scene: ClothOnTableScene, depth: RegisteredDepthImage
) -> None:
    """
    Where the camera measured nothing, there is no world point.
    """
    depth.depth_image[100, 100] = 0

    assert depth.world_point(Point2D(x=100, y=100)) is None


def test_a_pixel_outside_the_image_sees_no_point(depth: RegisteredDepthImage) -> None:
    """
    A pixel the image does not hold has no world point.
    """
    assert depth.world_point(Point2D(x=-1, y=100)) is None


def test_a_color_pixel_is_looked_up_in_a_smaller_depth_image(
    scene: ClothOnTableScene,
) -> None:
    """
    A depth image of half the color image's resolution is looked up at half the pixel,
    with the camera model scaled to it.
    """
    cas = scene.cas()
    cas.depth_image = scene.depth_image()[::2, ::2]
    cas.color_to_depth_ratio = ColorToDepthRatio(horizontal=0.5, vertical=0.5)
    depth = RegisteredDepthImage.from_cas(cas)

    point = depth.world_point(Point2D(x=150, y=100))

    assert np.allclose(point.to_np()[:3], [0.5, 0.0, 0.0])
