"""
Tests for the garments cloth is made into.
"""

from __future__ import annotations

import numpy
import pytest

from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world_description.cloth import ClothSheet, GarmentFile

# %% the T-shirt


@pytest.fixture(scope="module")
def t_shirt() -> GarmentFile:
    """
    :return: The T-shirt, as its file lays it out.
    """
    return GarmentFile.T_SHIRT


def test_a_t_shirt_is_sewn_into_one_piece_open_at_the_hem_the_neck_and_the_cuffs(
    t_shirt: GarmentFile,
) -> None:
    """
    The panels are joined at every seam, so the surface is one piece whose only edges
    are the four openings: the hem, the neck and the two cuffs.
    """
    sheet = ClothSheet(
        name="t_shirt",
        surface=t_shirt.create_surface(HomogeneousTransformationMatrix()),
        mass=0.255,
    )

    assert len(sheet.starting_surface.split(only_watertight=False)) == 1
    assert len(sheet.starting_surface.outline().entities) == 4


def test_a_t_shirt_lies_where_it_is_placed(t_shirt: GarmentFile) -> None:
    """
    A garment placed at a pose is its file's layout moved and turned by that pose.
    """
    as_laid_out = ClothSheet(
        name="t_shirt",
        surface=t_shirt.create_surface(HomogeneousTransformationMatrix()),
        mass=0.255,
    )
    world_T_t_shirt = HomogeneousTransformationMatrix.from_xyz_rpy(
        1.0, -2.0, 0.8, yaw=numpy.pi / 2
    )

    placed = ClothSheet(
        name="t_shirt", surface=t_shirt.create_surface(world_T_t_shirt), mass=0.255
    )

    assert numpy.allclose(
        placed.starting_surface.vertices,
        as_laid_out.starting_surface.vertices @ world_T_t_shirt.to_np()[:3, :3].T
        + world_T_t_shirt.to_np()[:3, 3],
    )
